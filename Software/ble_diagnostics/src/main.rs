use btleplug::api::{Central, Manager as _, Peripheral, ScanFilter};
use btleplug::platform::Manager;
use bytemuck::{Pod, Zeroable};
use crossterm::{
  event::{self, Event, KeyCode},
  execute,
  terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use futures::stream::StreamExt;
use ratatui::{
  prelude::*,
  widgets::{Block, Borders, Paragraph},
};
use std::io;
use std::sync::{Arc, Mutex};
use std::time::Duration;

#[derive(Copy, Clone, Debug, Pod, Zeroable)]
#[repr(C, packed)]
struct TelemetryFrame {
  timestamp: u32,
  adc_data: [u8; 16],
  gyro: [i16; 3],
  accel: [i16; 3],
  battery_mv: u16,
}

struct AppState {
  frame: TelemetryFrame,
  status_message: String,
  packet_count: u64,
  bytes_per_second: f64,
  frequency_hz: f64,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
  enable_raw_mode()?;
  let mut stdout = io::stdout();
  execute!(stdout, EnterAlternateScreen)?;
  let backend = CrosstermBackend::new(stdout);
  let mut terminal = Terminal::new(backend)?;

  let shared_state = Arc::new(Mutex::new(AppState {
    frame: TelemetryFrame {
      timestamp: 0,
      adc_data: [0; 16],
      gyro: [0; 3],
      accel: [0; 3],
      battery_mv: 0,
    },
    status_message: String::from("Initializing Bluetooth local adapter..."),
    packet_count: 0,
    bytes_per_second: 0.0,
    frequency_hz: 0.0,
  }));

  let ble_state_handle = shared_state.clone();
  tokio::spawn(async move {
    if let Err(e) = run_ble_receiver(ble_state_handle.clone()).await {
      let mut lock = ble_state_handle.lock().unwrap();
      lock.status_message = format!("CRITICAL ERROR: {:?}", e);
    }
  });

  loop {
    terminal.draw(|f| {
      let state = shared_state.lock().unwrap();
      // FIX 1: Pass the full state reference here instead of just the inner frame
      ui_layout(f, &state);
    })?;

    if event::poll(Duration::from_millis(16))? {
      if let Event::Key(key) = event::read()? {
        if key.code == KeyCode::Char('q') {
          break;
        }
      }
    }
  }

  disable_raw_mode()?;
  execute!(terminal.backend_mut(), LeaveAlternateScreen)?;
  Ok(())
}

async fn run_ble_receiver(state_store: Arc<Mutex<AppState>>) -> Result<(), Box<dyn std::error::Error>> {
  let manager = Manager::new().await?;
  let adapters = manager.adapters().await?;
  let central = adapters.into_iter().next().ok_or("No Bluetooth adapters found")?;

  {
    let mut lock = state_store.lock().unwrap();
    lock.status_message = String::from("Scanning for Venturi_P4 broadcast beacons...");
  }

  central.start_scan(ScanFilter::default()).await?;
  tokio::time::sleep(Duration::from_secs(2)).await;

  let peripherals = central.peripherals().await?;
  let mut target_peripheral = None;

  for peripheral in peripherals {
    if let Ok(Some(properties)) = peripheral.properties().await {
      if let Some(name) = properties.local_name {
        if name.contains("Venturi_P4") {
          target_peripheral = Some(peripheral);
          break;
        }
      }
    }
  }

  let target_peripheral = target_peripheral.ok_or("Venturi module not found nearby")?;

  {
    let mut lock = state_store.lock().unwrap();
    lock.status_message = String::from("Target identified. Opening BLE handshake Link...");
  }

  target_peripheral.connect().await?;

  {
    let mut lock = state_store.lock().unwrap();
    lock.status_message = String::from("Connected. Querying internal GATT service table...");
  }

  target_peripheral.discover_services().await?;

  {
    let mut lock = state_store.lock().unwrap();
    lock.status_message = String::from("GATT resolved. Mapping notification descriptors...");
  }

  let characteristics = target_peripheral.characteristics();
  let telemetry_char = characteristics
    .iter()
    .find(|c| c.properties.contains(btleplug::api::CharPropFlags::NOTIFY))
    .ok_or("No notify-capable telemetry characteristic found on peripheral")?;

  target_peripheral.subscribe(telemetry_char).await?;

  let mut notification_stream = target_peripheral.notifications().await?;

  let mut last_update = std::time::Instant::now();
  let mut total_bytes_received = 0;
  let mut local_packet_counter = 0;

  while let Some(notification) = notification_stream.next().await {
    let incoming_bytes = &notification.value;
    let rx_len = incoming_bytes.len();

    local_packet_counter += 1;
    total_bytes_received += rx_len;

    let now = std::time::Instant::now();
    let elapsed = now.duration_since(last_update).as_secs_f64();

    let mut current_hz = 0.0;
    let mut current_bps = 0.0;

    if elapsed >= 0.5 {
      current_hz = local_packet_counter as f64 / elapsed;
      current_bps = total_bytes_received as f64 / elapsed;

      local_packet_counter = 0;
      total_bytes_received = 0;
      last_update = now;
    }

    let mut aligned_buffer = [0u8; std::mem::size_of::<TelemetryFrame>()];
    let bytes_to_copy = rx_len.min(aligned_buffer.len());
    aligned_buffer[..bytes_to_copy].copy_from_slice(&incoming_bytes[..bytes_to_copy]);

    if let Ok(unpacked) = bytemuck::try_from_bytes::<TelemetryFrame>(&aligned_buffer) {
      let mut lock = state_store.lock().unwrap();
      lock.frame = *unpacked;

      if elapsed >= 0.5 {
        lock.frequency_hz = current_hz;
        lock.bytes_per_second = current_bps;
      }
      lock.packet_count += 1;

      lock.status_message = format!(
        "Streaming Payloads Active! [OK] | Frame Count: #{}",
        lock.packet_count
      );
    }
  }
  Ok(())
}

// FIX 2: Altered function header signature to consume AppState reference directly
fn ui_layout(f: &mut Frame, state: &AppState) {
  let size = f.size();
  let data = &state.frame;
  let status_msg = &state.status_message;

  let timestamp = data.timestamp;
  let battery_mv = data.battery_mv;
  let gyro = data.gyro;
  let accel = data.accel;
  let adc_data = data.adc_data;

  // Master Vertical Layout Split
  let main_chunks = Layout::default()
    .direction(Direction::Vertical)
    .constraints([
      Constraint::Length(3),
      Constraint::Length(6), // Raised to 6 to safely display all 4 network metrics rows
      Constraint::Min(6),
    ])
    .split(size);

  let metric_chunks = Layout::default()
    .direction(Direction::Horizontal)
    .constraints([
      Constraint::Min(34), // Adjusted to preserve alignment boundary bounds
      Constraint::Min(50),
    ])
    .split(main_chunks[1]);

  // --- WIDGET 1: System Link Monitor Status Bar ---
  let status_style = if status_msg.contains("CRITICAL") || status_msg.contains("Error") {
    Style::default().fg(Color::Red).add_modifier(Modifier::BOLD)
  } else if status_msg.contains("[OK]") {
    Style::default().fg(Color::Green)
  } else {
    Style::default().fg(Color::Yellow)
  };

  let status_widget = Paragraph::new(format!("  {}", status_msg))
    .block(Block::default().title(" Link Infrastructure Monitor ").borders(Borders::ALL))
    .style(status_style);
  f.render_widget(status_widget, main_chunks[0]);

  // --- WIDGET 2: Power & Internal Engine Timers ---
  let voltage_color = if battery_mv < 3400 && battery_mv > 0 { Color::Red } else { Color::Magenta };

  // FIX 3: Replaced broken "state" references with valid local scoped fields
  let power_text = format!(
    "  Internal Timer : {} ms\n\
       Bus Voltage    : {} mV\n\
       Link Frequency : {:.1} Hz\n\
       Data Data Rate : {:.2} KB/s",
    timestamp,
    battery_mv,
    state.frequency_hz,
    state.bytes_per_second / 1024.0
  );

  let power_widget = Paragraph::new(power_text)
    .block(Block::default().title(" Core Power & Speed ").borders(Borders::ALL).border_style(Style::default().fg(Color::Cyan)))
    .style(Style::default().fg(voltage_color));
  f.render_widget(power_widget, metric_chunks[0]);

  // --- WIDGET 3: Kinematics Inertial Measurement Unit (IMU) ---
  let imu_text = format!(
    "  Gyroscope Data  => X: {:5} | Y: {:5} | Z: {:5}\n  Accelerometer   => X: {:5} | Y: {:5} | Z: {:5}",
    gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]
  );
  let imu_widget = Paragraph::new(imu_text)
    .block(Block::default().title(" Kinematics (6-DoF IMU) ").borders(Borders::ALL).border_style(Style::default().fg(Color::Cyan)));
  f.render_widget(imu_widget, metric_chunks[1]);

  // --- WIDGET 4: High-Density 16-Channel Continuous Bar (2 Rows Thick) ---
  let get_block_char = |val: u8| match val {
    0..=63    => "    ",
    64..=127  => "░░░░",
    128..=191 => "▒▒▒▒",
    _         => "████",
  };

  let mut single_line = String::from("  [");
  for i in 0..16 {
    single_line.push_str(get_block_char(adc_data[i]));
  }
  single_line.push(']');

  let dual_row_display = format!("\n{}\n{}", single_line, single_line);

  let sensor_widget = Paragraph::new(dual_row_display)
    .block(Block::default().title(" 16-Channel Reflectance Array Alignment Map ").borders(Borders::ALL).border_style(Style::default().fg(Color::Yellow)));
  f.render_widget(sensor_widget, main_chunks[2]);
}