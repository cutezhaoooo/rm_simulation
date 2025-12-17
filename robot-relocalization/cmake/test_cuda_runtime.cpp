#include <cuda_runtime.h>
#include <iostream>

int main() {
    int deviceCount = 0;
    cudaError_t error = cudaGetDeviceCount(&deviceCount);
    
    if (error != cudaSuccess) {
        std::cerr << "CUDA error: " << cudaGetErrorString(error) << std::endl;
        return 1;
    }
    
    if (deviceCount == 0) {
        std::cerr << "No CUDA-capable devices found" << std::endl;
        return 1;
    }
    
    std::cout << "Found " << deviceCount << " CUDA device(s)" << std::endl;
    
    // Test basic CUDA functionality
    for (int i = 0; i < deviceCount; ++i) {
        cudaDeviceProp prop;
        error = cudaGetDeviceProperties(&prop, i);
        if (error != cudaSuccess) {
            std::cerr << "Failed to get device properties for device " << i << std::endl;
            return 1;
        }
        std::cout << "Device " << i << ": " << prop.name << std::endl;
    }
    
    return 0;
}