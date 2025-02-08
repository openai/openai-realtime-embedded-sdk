set(COMMON_SRC "webrtc.cpp" "main.cpp" "http.cpp")

if(IDF_TARGET STREQUAL linux)
	idf_component_register(
		SRCS ${COMMON_SRC}
		REQUIRES peer esp_http_client)
else()
	idf_component_register(
		SRCS ${COMMON_SRC} "wifi.cpp" "media.cpp" "media_gmf.cpp"
		REQUIRES driver esp_wifi nvs_flash peer esp_psram esp_timer esp_http_client)
endif()

idf_component_get_property(lib peer COMPONENT_LIB)
target_compile_options(${lib} PRIVATE -Wno-error=restrict)
target_compile_options(${lib} PRIVATE -Wno-error=stringop-truncation)

idf_component_get_property(lib sepfy__srtp COMPONENT_LIB)
target_compile_options(${lib} PRIVATE -Wno-error=incompatible-pointer-types)

set(TARGET_COMPONENT "")

idf_build_get_property(build_components BUILD_COMPONENTS)
foreach(COMPONENT ${build_components})
	if(COMPONENT MATCHES "gmf_core")
		set(TARGET_COMPONENT ${COMPONENT})
		break()
	endif()
endforeach()

if(TARGET_COMPONENT STREQUAL "")
	target_compile_definitions(${COMPONENT_TARGET} PRIVATE "-D CONFIG_OPENAI_WITH_GMF=0")
else()
	idf_component_get_property(lib ${TARGET_COMPONENT} COMPONENT_LIB)
	target_compile_options(${lib} PRIVATE -Wno-error=incompatible-pointer-types)
	target_compile_definitions(${COMPONENT_TARGET} PRIVATE "-D CONFIG_OPENAI_WITH_GMF=1")
endif()
