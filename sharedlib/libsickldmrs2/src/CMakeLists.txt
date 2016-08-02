
SET(sickldmrs_LIB_SRCS
  vpSickLDMRS.cpp
  vpTime.cpp
)

add_library(sickldmrs SHARED ${sickldmrs_LIB_SRCS})

target_link_libraries(sickldmrs rt)

install(TARGETS sickldmrs DESTINATION lib)
