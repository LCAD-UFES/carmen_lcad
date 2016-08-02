SET(sickldmrstest_SRCS
   sickldmrstest.cpp
)

add_executable(sickldmrstest ${sickldmrstest_SRCS})

target_link_libraries(sickldmrstest sickldmrs m)

install(TARGETS sickldmrstest DESTINATION bin)

