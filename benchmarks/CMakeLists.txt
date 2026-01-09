add_executable(benchmark_av
    benchmark_main.cpp
)

target_link_libraries(benchmark_av PRIVATE
    av
    benchmark::benchmark
    benchmark::benchmark_main
)
