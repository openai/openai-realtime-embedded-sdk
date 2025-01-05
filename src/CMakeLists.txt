set(COMMON_SRC "webrtc.cpp" "main.cpp" "http.cpp" "media.cpp")

if(IDF_TARGET STREQUAL linux)
	idf_component_register(
		SRCS ${COMMON_SRC} "platform_linux.cpp"
		REQUIRES peer esp-libopus esp_http_client)
	target_link_libraries(${COMPONENT_LIB} PRIVATE pulse pulse-simple)
	target_link_libraries(${COMPONENT_LIB} PRIVATE "-lbsd")
else()
	idf_component_register(
		SRCS ${COMMON_SRC} "platform_esp32s3.cpp" "wifi.cpp"
		REQUIRES driver esp_wifi nvs_flash peer esp_psram esp-libopus esp_http_client)
endif()

idf_component_get_property(lib peer COMPONENT_LIB)
target_compile_options(${lib} PRIVATE -Wno-error=restrict)
target_compile_options(${lib} PRIVATE -Wno-error=stringop-truncation)

idf_component_get_property(lib srtp COMPONENT_LIB)
target_compile_options(${lib} PRIVATE -Wno-error=incompatible-pointer-types)

idf_component_get_property(lib esp-libopus COMPONENT_LIB)
target_compile_options(${lib} PRIVATE -Wno-error=maybe-uninitialized)
target_compile_options(${lib} PRIVATE -Wno-error=stringop-overread)
