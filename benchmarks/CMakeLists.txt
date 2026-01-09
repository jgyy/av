add_executable(benchmark_av
    benchmark_main.cpp
)

target_link_libraries(benchmark_av PRIVATE
    av
)

# Find benchmark library
find_package(benchmark QUIET)
if(benchmark_FOUND)
    target_link_libraries(benchmark_av PRIVATE
        benchmark::benchmark
        benchmark::benchmark_main
    )
endif()
