# Self_drive

중앙대학교 자율주행 모형차량 경진대회 2기 로켓라쿤팀

포트폴리오 용 깃헙 자료

# 사용 부품 리스트

|:부품명:|:수량:|:용도:|
|------|------|------|
|:emax eco 1106:|:2:|:구동모터:|
|:emax bullet 12A:|:2:|:BLDC esc:|
|:Raspberry pi 4:|:1:|:메인 MCU:|
|:Raspberry pi pico:|:1:|:센서 MCU:|
|:TSL1401:|:1:|:Line Scan:|
|:picam v2:|:1:|:Object Scan:|
|:SR-04:|:1:|Emergency Stop:|
|:2S Lipo battery:|:1:|:Motor Power:|
|:5V 4A UPS:|:1:|:MCU Power:|
|:18650 2000mA:|:2:|MCU Power:|
|:5V LED:|:2:|:mission:|

# 주행로직

TSL1401으로 라인을 감지, 중앙값으로부터의 거리를 Error으로 판단, PID 제어

좌우 라인이 점선인지 실선인지 실시간 감지

양측이 실선이면 전방 장애물 감지하여 비상정지

양측 중 한 선이 점선이면 추월진행

주행 트랙의 조도를 TSL1401으로 감지하여 LED on/off 진행

우측 공백이 일정 시간 이상 있으면 주차선으로 인식, 주차 진행

주차 이후 카메라 인식, LLM으로 신호등 인식

# 코드관련

rpi4 : C++ //vscode
pico : Arduino IDE

# RPI4

C++ 기반으로 코드 작성, opencv를 이용하여 카메라 촬영 후 pico 로부터 신호 입력받으면 LLM api이용 신호등 인식

키입력을 받아 pico에 주행 모드 전송

# PICO

Arduino IDE이용 듀얼코어 사용

0번코어 -> SR-04 측정
1번코어 -> TSL1401 관련 연산, PID 제어 연산, RPi4 의 신호를 읽어 주행모드 결정, RPi4에 LLM 동작신호 전달