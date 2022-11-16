

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN")

set(TESTNAMES testTFromDH testFwdKin testJacobian)


foreach(test ${TESTNAMES})
	add_executable(${test} ${test}.cpp) 
	target_link_libraries(${test} RX150Interface ${Boost_LIBRARIES})
	add_test(${test} ${CMAKE_CURRENT_BINARY_DIR}/${test} ${TEST_OUTPUT_FLAGS})
endforeach()



