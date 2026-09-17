// Abstract (Deutsch & English)

#heading(numbering: none, outlined: false)[Abstract -- Deutsch]

Die vorliegende Diplomarbeit beschäftigt sich mit der Entwicklung eines ENN-gesteuerten autonomen Modellfahrzeugs. Herkömmliche Ansätze für autonomes Fahren setzen auf eine Vielzahl teurer Sensoren wie LiDAR oder Kameras sowie leistungsstarke Rechenhardware. Ziel dieses Projekts ist es, einen alternativen Weg aufzuzeigen, bei dem die Komplexität von der Hardware in die Software verlagert wird.

Dazu wurde zunächst eine 2D-Simulation in Unity erstellt, die das Fahrzeug mit Differentialantrieb und seiner Sensorik -- bestehend aus fünf Infrarot-Abstandssensoren, einem Gyrosensor und Drehzahlencodern -- realitätsnah abbildet. Über eine TCP-Socket-Brücke zwischen Unity und Python wurde anschließend ein evolutionäres neuronales Netz (ENN) mittels NEAT-Neuroevolution über hunderte Generationen trainiert. Das so optimierte Netz wurde abschließend in C-Code umgewandelt und auf den Mikrocontroller des physischen Fahrzeugs übertragen.

Diese "Sim-to-Real"-Pipeline demonstriert, dass KI-gesteuerte Autonomie auch mit minimaler, kostengünstiger Hardware robust realisiert werden kann. Während der Entwicklung konnten wir unsere Fähigkeiten in den Bereichen maschinelles Lernen, Simulationstechnik, eingebettete Systeme und Projektmanagement wesentlich erweitern.

#v(1cm)

#heading(numbering: none, outlined: false)[Abstract -- English]

This diploma thesis presents the development of an autonomous model vehicle controlled by an evolutionary neural network (ENN). Conventional approaches to autonomous driving rely on a multitude of expensive sensors such as LiDAR or cameras, along with high-performance computing hardware. The aim of this project is to demonstrate an alternative approach in which complexity is shifted from hardware to software.

To this end, a 2D simulation was created in Unity that realistically models the vehicle with differential drive and its sensor suite -- consisting of five infrared distance sensors, a gyroscope, and rotary encoders. A TCP socket bridge between Unity and Python was then used to train an evolutionary neural network via NEAT neuroevolution over hundreds of generations. The resulting optimized network was finally converted into C code and deployed onto the microcontroller of the physical vehicle.

This "Sim-to-Real" pipeline demonstrates that AI-driven autonomy can be robustly achieved even with minimal, low-cost hardware. Throughout the development process, we significantly expanded our skills in the areas of machine learning, simulation engineering, embedded systems, and project management.

#pagebreak()
