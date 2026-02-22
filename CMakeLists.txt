cmake_minimum_required(VERSION 3.10)

# Устанавливаем переменную TARGET равной имени проекта,
# чтобы потом везде подставлять
set(TARGET topology-gen)

# Задаем имя проекта
project(${TARGET})

# Путь, куда надо сложить собранное
set(EXECUTABLE_OUTPUT_PATH "../bin")

# Пути к заголовкам и сорцам
file(GLOB INCLUDES "./include/*.h" "./include/*.hpp")
file(GLOB SOURCES "./src/*.cpp")

# Компиляция приложения
add_executable(${TARGET} ${INCLUDES} ${SOURCES})

# Указываем пути поиска заголовков
target_include_directories (${TARGET} PRIVATE ./include)
