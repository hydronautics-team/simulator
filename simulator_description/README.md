# simulator_description

Пакет `simulator_description` содержит описание моделей для симулятора и экспортирует пути ресурсов для Gazebo.

## Назначение

Пакет отвечает за:
- хранение SDFormat-моделей в каталоге `models/`;
- установку моделей в `share/simulator_description/models`;
- экспорт environment hooks, чтобы Gazebo находил ресурсы пакета.

## Состав пакета

- `models/` — модели симулятора:
  - `car/`
  - `diff_drive/`
  - `rrbot/`
- `hooks/`
  - `simulator_description.dsv.in`
  - `simulator_description.sh.in`
- `CMakeLists.txt`
- `package.xml`

## Установка

В `CMakeLists.txt` устанавливается каталог моделей:

- источник: `models/`
- назначение: `share/${PROJECT_NAME}/models`

Также подключены environment hooks через `ament_environment_hooks(...)`.

## Использование

Пакет используется другими пакетами стека (в первую очередь launch/simulation), когда требуется доступ к моделям робота и связанных ресурсов в Gazebo.

## Добавление новой модели

1. Создать новую папку в `models/<model_name>/`.
2. Добавить `model.sdf` и `model.config` (и при необходимости `meshes/`, `materials/`).
3. Проверить, что модель корректно разрешается через установленные пути ресурсов.
4. Использовать модель в launch/world-файлах симулятора.