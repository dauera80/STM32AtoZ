# STM32CubeIDE를 이용한 STM32 따라하기
이 깃허브는 [STM32CubeIDE를 이용한 STM32 따라하기](http://www.kyobobook.co.kr/product/detailViewKor.laf?ejkGb=KOR&mallGb=KOR&barcode=9791165392659&orderClick=LEa&Kc=)에 포함된 실습 코드를 담고 있습니다.
<a href="http://www.kyobobook.co.kr/product/detailViewKor.laf?ejkGb=KOR&mallGb=KOR&barcode=9791165392659&orderClick=LEa&Kc="><img src="https://image.yes24.com/goods/90611902/800x0" border="0" width="70%"></a>

# 개발 환경 및 실습 보드
|IDE             |Target board         |External board        |FW version     |
|:--------------:|:-------------------:|:--------------------:|:-------------:|
|<center>[STM32CubeIDE 1.3.0](https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-ides/stm32cubeide.html)</center>|<center>[NUCLEO64](https://www.devicemart.co.kr/goods/view?no=1346033)</center>|<center>[NUCLEOEVB](https://www.devicemart.co.kr/goods/view?no=12545343)</center>|F1_V1.8.0
|<img src="https://encrypted-tbn0.gstatic.com/images?q=tbn%3AANd9GcRGhTJJ8gbDkFXe_0Md4uHzcOrr558cVxnCbw&usqp=CAU" border="0" width="200">|<img src="https://user-images.githubusercontent.com/67400790/86118039-0cae3800-bb0b-11ea-85ba-a246ab6d0b4b.png" border="0" width="100">|<img src="https://user-images.githubusercontent.com/67400790/86117881-c953c980-bb0a-11ea-8c28-f9621f89737b.jpg" border="0" width="200">
  
* NUCLEO64 및 NUCELOEVB 보드 이미지
<img src="https://user-images.githubusercontent.com/67400790/86118831-6105e780-bb0c-11ea-80d1-72107f9bb4ff.jpg" border="0" width="85%">  
  
# 목차
**1. STM32 요약**  
1.1. STMicroelectronics STM32 현황  
1.2. STM32 32-bit MCU Family 소개  
1.3. STM32 개발 보드 종류  
1.4. STM32 에코 시스템   
  
**2. 개발 환경 구축**  
2.1. 실습 보드 소개  
2.2. STM32CubeIDE 설치  
2.3. 터미널 통신 프로그램 설치  
  
**3. STM32CubeIDE 프로젝트 시작하기**  
3.1. LED Blink [[예제]](https://github.com/dauera80/stm32atoz/tree/master/LED_Blink)  
3.2. printf 시리얼 디버깅 [[예제]](https://github.com/dauera80/stm32atoz/tree/master/USART_printf)  
  
**4. Peripheral 예제**  
4.1. EXTI [[예제]](https://github.com/dauera80/stm32atoz/tree/master/EXTI)  
4.2. TIM_TimeBase [[예제]](https://github.com/dauera80/stm32atoz/tree/master/TIM_TimeBase)  
4.3. ADC  [[예제]](https://github.com/dauera80/stm32atoz/tree/master/ADC_TemperatureSensor)  
4.4. WWDG [[예제]](https://github.com/dauera80/stm32atoz/tree/master/WWDG)  
4.5. USART [[예제]](https://github.com/dauera80/stm32atoz/tree/master/USART_rb)  
4.6. TIM_PWM [[예제]](https://github.com/dauera80/stm32atoz/tree/master/TIM_PWM)  
  
**5. NUCLEOEVB 보드를 이용한 실습**  
5.1. GPIO [[예제]](https://github.com/dauera80/stm32atoz/tree/master/GPIO)  
5.2. EXTI  
>5.2.1 PushButton [[예제]](https://github.com/dauera80/stm32atoz/tree/master/EXTI_PushButton)  
>5.2.2 Encoder [[예제]](https://github.com/dauera80/stm32atoz/tree/master/EXTI_Encoder)  

5.3. ADC  
>5.3.1 Polling Sequencer [[예제]](https://github.com/dauera80/stm32atoz/tree/master/ADC_Polling_Sequencer)  
>5.3.2 Interrupt [[예제]](https://github.com/dauera80/stm32atoz/tree/master/ADC_Interrupt)  
>5.3.3 DMA [[예제]](https://github.com/dauera80/stm32atoz/tree/master/ADC_DMA_Sequencer)  

5.4. TIM  
>5.4.1 Buzzer [[예제]](https://github.com/dauera80/stm32atoz/tree/master/TIM_Buzzer)  
>5.4.2 Servo [[예제]](https://github.com/dauera80/stm32atoz/tree/master/TIM_Servo)  
>5.4.3 RGB LED [[예제]](https://github.com/dauera80/stm32atoz/tree/master/TIM_RGBLED)  

5.5. I2C 
>5.5.1 Temperature & Humidity [[예제]](https://github.com/dauera80/stm32atoz/tree/master/I2C_HDC1080)  
>5.5.2 EEPROM [[예제]](https://github.com/dauera80/stm32atoz/tree/master/I2C_EEPROM)  

5.6. SPI [[예제]](https://github.com/dauera80/stm32atoz/tree/master/SPI_DAC)  

5.7. CLCD [[예제]](https://github.com/dauera80/stm32atoz/tree/master/CLCD)  
5.8. Application  
>5.8.1 Demo1 [[예제]](https://github.com/dauera80/stm32atoz/tree/master/Demo1)  
>5.8.2 Demo2 [[예제]](https://github.com/dauera80/stm32atoz/tree/master/Demo2)  

**6. 번외**  
6.1. RTC  
>6.1.1 RTC [[예제]](https://github.com/dauera80/stm32atoz/tree/master/RTC)  
>6.1.2 RTC_VBAT [[예제]](https://github.com/dauera80/stm32atoz/tree/master/RTC_VBAT)  
>6.1.3 RTC_VBAT_F411 [[예제]](https://github.com/dauera80/stm32atoz/tree/master/RTC_VBAT_F411)  


