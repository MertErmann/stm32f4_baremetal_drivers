# 🛠️ STM32F4 Bare-Metal Driver Development

## 📖 Introduction
This repository is part of my personal journey to **learn and implement low-level (bare-metal) drivers** for STM32F4 microcontrollers (STM32F407 focus).  
Instead of using HAL or Cube libraries, I directly program the **peripheral registers** (RCC, GPIO, EXTI, SYSCFG, NVIC).  
The goal is to build a **reusable driver set**, step by step, while gaining a deep understanding of how the hardware works.  

---

## 📌 About
This repository contains **register-level drivers** for STM32F4.  
It starts with GPIO and will gradually expand to SPI, I2C, USART, SysTick, and sensor drivers.

---

## ✅ Currently Implemented
- **GPIO Driver**
  - Pin modes (Input, Output, Alternate, Analog)
  - Speed / Output type / Pull-up / Pull-down / Alternate function
  - EXTI (rising, falling, both edges) configuration
  - NVIC interrupt configuration & priority
  - Pin/Port read & write, **BSRR** atomic set/reset, toggle

---

## 🧭 Roadmap
- [x] GPIO driver  
- [ ] SPI driver (full-duplex master, CPOL/CPHA, NSS handling)  
- [ ] I2C driver (master TX/RX, start/stop/ack, burst transfer, 7-bit addressing)  
- [ ] USART driver (TX/RX, baud rate calculation, interrupt-based communication)  
- [ ] SysTick driver (ms delay, timing services)  
- [ ] **Sensor drivers** built on top of LL APIs (MPU6050, DS18B20, OPT301, …)  

---

##  License
MIT License. See `LICENSE` file.

---

##  Author
**Mert ERMAN**  
- Electrical & Electronics Engineer  
- Embedded Systems & Bare-Metal Programming Enthusiast  
- Developing low-level drivers on STM32F4 as part of a learning journey  

📫 Connect with me on [LinkedIn](https://www.linkedin.com/in/merterman/)

