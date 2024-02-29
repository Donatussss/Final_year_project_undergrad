# Gas pressure, flowrate and concentration monitoring for automated electrolysis

This is my final year undergrad project

### Project Aims
- Meet oxygen demand in hospitals
- Realise an affordable device to generate oxygen

### Equipment
- STM32F411 Black Pill development board. [Source](https://store.nerokas.co.ke/index.php?route=product/product&product_id=2421&search=Stm32f4)
- Gas pressure sensor JPG25A5P1-16BarG. [Source](https://www.alibaba.com/product-detail/Laboratory-12-volt-0-30bar-exhaust_1600609090836.html)
- Gas flowrate and concentration sensor. [Source](https://www.alibaba.com/product-detail/Ultrasonic-oxygen-sensor-for-Oxygen-Concentrator_1600161704012.html?spm=a2700.galleryofferlist.normal_offer.d_title.3f3366e7txqYtu)
- Solid state relay. [Source](https://www.pixelelectric.com/electronic-modules/miscellaneous-modules/relay-switch/5v-2-ch-omron-ssr-solid-relay-module/)
- Gas reservoir
- Display


### Basic operation
The mcu will be monitoring the gas flowrate and concentration as well as pressure in the reservoir. It does the following actions based on certain conditions
- Gas pressure above threshold & gas flowrate present - Turn off electrolysis electrodes
- Gas pressure below threshold & gas flowrate not present - Turn on electrolyis electrodes