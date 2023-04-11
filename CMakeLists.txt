set(srcs 
 "Adafruit_SharpMem.cpp"
)

# If the project does not use a touch display component FT6X36-IDF can be removed or #commented
idf_component_register(SRCS ${srcs}      
                    REQUIRES "Adafruit-GFX"
                    driver esp_timer
                    INCLUDE_DIRS "include"
)