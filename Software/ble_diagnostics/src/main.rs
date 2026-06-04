use btleplug::api::{Central, Manager as _, Peripheral, ScanFilter, WriteType};
use btleplug::platform::Manager;
use bytemuck::{Pod, Zeroable};
use crossterm::{
    event::{self, Event, KeyCode},
    execute,
    terminal::{EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode},
};
use ratatui::widgets::{Cell, Row, Table, Wrap};
use ratatui::{
    prelude::*,
    widgets::{Block, Borders, Paragraph},
};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;
use tokio::sync::{Mutex, mpsc};
use uuid::Uuid;

const IP_CHAR_UUID: &str = "822c9530-9548-4375-8422-909247192312";

#[derive(Copy, Clone, Debug, Pod, Zeroable)]
#[repr(C, packed)]
struct TelemetryFrame {
  timestamp: u32,          // Offset 0
  steering: i32,           // Offset 4
  temperature: f32,        // Offset 8
  gyro: [i16; 3],          // Offset 12
  accel: [i16; 3],         // Offset 18
  battery_mv: u16,         // Offset 24
  update_speed: u16,       // Offset 26
  adc_data: [u8; 16],      // Offset 28
} // Total: 44 Bytes. Perfect 1:1 match!

struct AppState {
    frame: TelemetryFrame,
    status_message: String,
    target_ip: String,
    packet_count: u64,
    bytes_per_second: f64,
    frequency_hz: f64,
    raw_buffer: [u8; 44],
}

fn ui_layout(f: &mut ratatui::Frame, state: &AppState) {
  let size = f.size();
  let data = &state.frame;

  let (timestamp, battery_mv, gyro, accel, adc_data, steering, temperature, update_rate) =
    (data.timestamp, data.battery_mv, data.gyro, data.accel, data.adc_data, data.steering, data.temperature, data.update_speed);


  let min_voltage = 6600.0;
  let max_voltage = 8400.0;
  let battery_pct = ((battery_mv as f64 - min_voltage) / (max_voltage - min_voltage) * 100.0).clamp(0.0, 100.0);

  let bat_height = 7;
  let bat_width = 30;

  // Calculate how many characters wide the fill should be for EVERY row
  let filled_width = (battery_pct / 100.0 * bat_width as f64).round() as usize;

  let bat_body = (0..bat_height)
    .map(|i| {
      // Fill this specific row up to the filled_width
      let fill = "█".repeat(filled_width);
      let empty = " ".repeat(bat_width - filled_width);

      // Determine the right-side cap (keep your existing cap logic)
      let right_cap = if i == bat_height / 2 - 1 {
        "└┐"
      } else if i == bat_height / 2 + 1 {
        "┌┘"
      } else if (i < bat_height / 2 + 1 && i > bat_height / 2 - 1) {
        " │"
      } else {
        "│ "
      };

      format!("│{}{}{}", fill, empty, right_cap)
    })
    .collect::<Vec<String>>()
    .join("\n");

  let battery_display = format!(
    "┌{}┐\n{}\n└{}┘",
    "─".repeat(bat_width),
    bat_body,
    "─".repeat(bat_width)
  );
  // --- Layout Definition ---
  let main_chunks = Layout::default()
    .direction(Direction::Vertical)
    .constraints([
      Constraint::Length(3),  // Status Bar
      Constraint::Length(12), // Metrics & IMU Row
      Constraint::Length(11), // Battery & Sensors Row
      Constraint::Min(3),     // Hex Dump
    ])
    .split(size);

  let top_row = Layout::default()
    .direction(Direction::Horizontal)
    .constraints([Constraint::Percentage(40), Constraint::Percentage(60)])
    .split(main_chunks[1]);

  let bottom_row = Layout::default()
    .direction(Direction::Horizontal)
    .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
    .split(main_chunks[2]);

  // --- Widget Construction ---

  // 1. Status
  let status_widget = Paragraph::new(format!(
    " Status: {} | Target ESP: {} | Press 'r' to retry, 'q' to quit",
    state.status_message, state.target_ip
  ))
    .style(Style::default().fg(Color::Yellow))
    .block(Block::default().title(" Link Infrastructure Monitor ").borders(Borders::ALL).border_style(Style::default().fg(Color::Cyan)));

  // 2. Metrics Table
  let power_rows = [
    Row::new(vec![Cell::from("Internal Timer"), Cell::from(format!("{} ms", timestamp))]),
    Row::new(vec![Cell::from("Battery"), Cell::from(format!("{} mV ({:.0}%)", battery_mv, battery_pct))]),
    Row::new(vec![Cell::from("Link Freq"), Cell::from(format!("{:.1} Hz", state.frequency_hz))]),
    Row::new(vec![Cell::from("Data Rate"), Cell::from(format!("{:.2} KB/s", state.bytes_per_second / 1024.0))]),
    Row::new(vec![Cell::from("Temperature"), Cell::from(format!("{:.2}°C", temperature))]),
    Row::new(vec![Cell::from("Update Rate"), Cell::from(format!("{}us", update_rate))]),

  ];
  let power_table = Table::new(power_rows, [Constraint::Percentage(50), Constraint::Percentage(50)])
    .header(Row::new(vec!["Metric", "Value"]).style(Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)))
    .block(Block::default().title(" Core Power & Speed ").borders(Borders::ALL).border_style(Style::default().fg(Color::Green)));

  // 3. IMU Table
  let imu_rows = [
    Row::new(vec![Cell::from("X"), Cell::from(gyro[0].to_string()), Cell::from(accel[0].to_string())]),
    Row::new(vec![Cell::from("Y"), Cell::from(gyro[1].to_string()), Cell::from(accel[1].to_string())]),
    Row::new(vec![Cell::from("Z"), Cell::from(gyro[2].to_string()), Cell::from(accel[2].to_string())]),
    Row::new(vec![Cell::from("STEER").fg(Color::Cyan), Cell::from(steering.to_string()).fg(Color::Cyan), Cell::from("-")]),
  ];
  let imu_table = Table::new(imu_rows, [Constraint::Percentage(20), Constraint::Percentage(40), Constraint::Percentage(40)])
    .header(Row::new(["Axis", "Gyro", "Accel"].map(|h| Cell::from(h).fg(Color::Yellow).add_modifier(Modifier::BOLD))))
    .block(Block::default().title(" Kinematics (6-DoF IMU) ").borders(Borders::ALL).border_style(Style::default().fg(Color::Magenta)));

  // 4. Sensors
  let mut spans = vec![Span::raw("  [")];
  for val in adc_data {
    let text = match val {
      0..=63 => "    ", 64..=127 => "▒▒▒▒", 128..=191 => "▓▓▓▓", _ => "████"
    };
    spans.push(Span::styled(text, Style::default().fg(Color::Blue)));
  }
  spans.push(Span::raw("]"));
  let sensor_widget = Paragraph::new(Line::from(spans)).alignment(Alignment::Center)
    .block(Block::default().title(" 16-Channel Reflectance Array ").borders(Borders::ALL));

  let battery_widget = Paragraph::new(battery_display).alignment(Alignment::Center)
    .block(Block::default().title(" System Power ").borders(Borders::ALL));

  let hex_layout = Layout::default()
    .direction(Direction::Horizontal)
    .constraints([
      Constraint::Fill(1),
      Constraint::Length(44 * 3), // Each byte is 2 chars + 1 space
      Constraint::Fill(1),
    ])
    .split(main_chunks[3].inner(&Margin::new(1, 1))); // Margin to fit inside the Block

  // 2. Build Table Data
  let indices: Vec<String> = (0..44).map(|i| format!("{:02X}", i)).collect();
  let hex_values: Vec<String> = state.raw_buffer.iter().map(|b| format!("{:02X}", b)).collect();

  let hex_table = Table::new(
    [
      Row::new(indices),   // Top row: Indices
      Row::new(hex_values), // Bottom row: Hex Data
    ],
    vec![Constraint::Length(2); 44] // 44 columns, each 2 chars wide
  )
    .block(
      Block::default()
        .title(" Raw Packet Hex Dump ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Gray))
    );

  f.render_widget(hex_table, hex_layout[1]);
  // --- Rendering ---
  f.render_widget(status_widget, main_chunks[0]);
  f.render_widget(power_table, top_row[0]);
  f.render_widget(imu_table, top_row[1]);
  f.render_widget(sensor_widget, bottom_row[0]);
  f.render_widget(battery_widget, bottom_row[1]);

}

async fn provision_ip_to_esp() -> Result<String, Box<dyn std::error::Error + Send + Sync>> {
    let local_ip = local_ip_address::local_ip()?.to_string();
    let manager = Manager::new().await?;
    let adapters = manager.adapters().await?;
    let central = adapters.into_iter().next().ok_or("No Bluetooth adapters")?;

    central.start_scan(ScanFilter::default()).await?;
    tokio::time::sleep(Duration::from_secs(2)).await;

    for p in central.peripherals().await? {
        if p.properties()
            .await?
            .and_then(|prop| prop.local_name)
            .unwrap_or_default()
            == "Venturi_P4"
        {
            p.connect().await?;
            p.discover_services().await?;
            if let Some(char) = p
                .characteristics()
                .into_iter()
                .find(|c| c.uuid == Uuid::parse_str(IP_CHAR_UUID).unwrap())
            {
                // WRITE the local IP to the ESP32
                p.write(&char, local_ip.as_bytes(), WriteType::WithResponse)
                    .await?;
                return Ok(local_ip);
            }
        }
    }
    Err("ESP32 not found".into())
}

async fn run_udp_receiver(state_store: Arc<Mutex<AppState>>, mut reset_rx: mpsc::Receiver<()>) {
    loop {
        // Provision the IP and get back what we sent
        let my_ip = match provision_ip_to_esp().await {
            Ok(ip) => ip,
            Err(_) => {
                tokio::time::sleep(Duration::from_secs(2)).await;
                continue;
            }
        };

        {
            let mut state = state_store.lock().await;
            state.target_ip = my_ip;
            state.status_message = "Provisioning complete. Waiting for UDP...".into();
        }

        let socket = UdpSocket::bind("0.0.0.0:5005").await.unwrap();
        let mut buf = [0u8; std::mem::size_of::<TelemetryFrame>()];
        let mut last_update = Instant::now();
        let mut stats = (0, 0);

        loop {
            tokio::select! {
                _ = reset_rx.recv() => break,
                Ok((len, _addr)) = socket.recv_from(&mut buf) => {
                    if len == buf.len() {
                    let frame = *bytemuck::from_bytes::<TelemetryFrame>(&buf);
                    let mut state = state_store.lock().await;
                    state.frame = frame;
                    state.raw_buffer = buf; // Update the raw buffer

                        stats.0 += len; stats.1 += 1;
                        if last_update.elapsed().as_secs_f64() >= 0.5 {
                            state.frequency_hz = stats.1 as f64 / last_update.elapsed().as_secs_f64();
                            state.bytes_per_second = stats.0 as f64 / last_update.elapsed().as_secs_f64();
                            stats = (0, 0); last_update = Instant::now();
                        }
                    }
                }
            }
        }
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    enable_raw_mode()?;
    let mut stdout = std::io::stdout();
    execute!(stdout, EnterAlternateScreen)?;
    let mut terminal = Terminal::new(CrosstermBackend::new(stdout))?;
    let state = Arc::new(Mutex::new(AppState {
        frame: unsafe { std::mem::zeroed() },
        status_message: "BLE Discovering...".into(),
        target_ip: "Unknown".into(),
        packet_count: 0,
        bytes_per_second: 0.0,
        frequency_hz: 0.0,
        raw_buffer: [0; 44],
    }));

    let (tx, rx) = mpsc::channel(1);
    tokio::spawn(run_udp_receiver(state.clone(), rx));

    loop {
        let current_state = state.lock().await;
        terminal.draw(|f| ui_layout(f, &current_state))?;
        drop(current_state);
        if event::poll(Duration::from_millis(16))? {
            if let Event::Key(k) = event::read()? {
                if k.code == KeyCode::Char('q') {
                    break;
                }
                if k.code == KeyCode::Char('r') {
                    let _ = tx.try_send(());
                }
            }
        }
    }
    disable_raw_mode()?;
    execute!(terminal.backend_mut(), LeaveAlternateScreen)?;
    Ok(())
}
