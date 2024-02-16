# 모든 실행파일등에 대한 리스트 저장
file(GLOB EXE_LIST "*.cpp")

# 각 실행파일의 링킹을 위한 반복작업 수행
foreach(SRC_FILE ${EXE_LIST})
    # 
    get_filename_component(EXE_NAME ${SRC_FILE} NAME_WE)  # Get the file name without extension

    MESSAGE("creating TARGET_EXE_NAME2: '${EXE_NAME}'")

    # 실행파일의 위치 변경
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/bin)
    
    # 실행파일 생성
    add_executable(${EXE_NAME} ${SRC_FILE})
    # 라이브러리 Linkning 
    target_link_libraries(${EXE_NAME} mylib)
    # 라이브러리 헤더 파일 위치 명시
    target_include_directories(${EXE_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/inc)
endforeach()