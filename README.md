# Optical Link Monitor

Terminal-based C++17 program that monitors a simulated optical module using a producer thread, a monitor thread and a thread-safe queue.

Build: g++ -std=c++17 -Wall -Wextra -pthread optical_monitor.cpp -o monitor
Run: ./monitor --duration 10 --fault-rate 0.2
Test: ./monitor --test

Note: optical values are simulated.
