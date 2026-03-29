# Архитектурная схема simulator

## 1) Актуальные модули

- **simulator_launch**  
  Оркестрация запуска и профили `launch`.

- **stingray_interfaces**  
  Контракты `msg/srv/action` для межмодульного взаимодействия.

- **simulator_description**  
  Модели/ресурсы робота для симуляции.

- **simulator_simulation**  
  Миры и ресурсы окружения Gazebo.

- **simulator_gazebo_plugins**  
  C++ плагины Gazebo.

- **simulator_perception**  
  Python-ноды обработки данных и адаптеры (`converter` и др.).

## 2) Поток данных

1. `simulator_launch` запускает симуляцию и прикладные ноды.
2. `simulator_simulation` + `simulator_gazebo_plugins` формируют симуляционные данные.
3. `simulator_perception` обрабатывает входы и публикует результат.
4. Контракты обмена фиксируются в `stingray_interfaces`.

## 3) Логическая диаграмма

```text
simulator_launch
      |
      v
simulator_simulation <---- simulator_description
      |
      v
simulator_gazebo_plugins
      |
      v
simulator_perception
      |
      v
stingray_interfaces
```

## 4) Контрактные namespace

- **Topics**  
  `/simulator/sensors/*`  
  `/simulator/perception/*`  
  `/simulator/state/*`  
  `/simulator/control/*`

- **Services**  
  `/simulator/control/*`  
  `/simulator/system/*`

- **Actions**  
  `/simulator/mission/*`

## 5) Правила

- Не использовать legacy-имена в коде, launch и конфигах.
- Все интеграции проводить через явные контракты `stingray_interfaces`.
- Имена файлов и launch должны отражать назначение (без неинформативных названий).