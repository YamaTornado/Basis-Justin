function(fc_add_test name)
    add_executable(${name} ${name}.c)
    target_link_libraries(${name} PRIVATE fc_core)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

fc_add_test(test_altitude_filter)
fc_add_test(test_flight_state)
fc_add_test(test_framing)
