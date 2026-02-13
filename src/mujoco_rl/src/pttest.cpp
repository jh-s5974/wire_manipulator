#include <torch/script.h> // PyTorch C++ API (LibTorch) 헤더
#include <iostream>
#include <vector>
#include <memory>
#include <chrono>

int main() {
    // -------------------------------------------------------------------
    // 1. 모델 로드
    // -------------------------------------------------------------------
    torch::jit::script::Module module;
    try {
        const std::string model_path = "/home/khrlab/mujoco_ws/src/mujoco_rl/policy/policy.pt"; 
        module = torch::jit::load(model_path);
    } 
    catch (const c10::Error& e) {
        std::cerr << "Error loading the model:\n" << e.what() << std::endl;
        return -1;
    }
    std::cout << "Model loaded successfully." << std::endl;

    // -------------------------------------------------------------------
    // 2. 입력 데이터 준비 (총 47개)
    // -------------------------------------------------------------------
    std::vector<float> clock_input = {-0.9022, -0.4314};                           // 2개
    std::vector<float> ang_vel_input = { 0.2747, -0.0520,  0.0722};                 // 3개
    std::vector<float> grv_input = {  0.0556, -0.0963, -0.9938};                      // 3개
    std::vector<float> com_input = {-0.8909, -0.2202, -0.1378};                        // 3개
    std::vector<float> pos_input{0.0886, -0.0061,  0.0334,  0.0229, -0.2085, -0.5420,  0.8875,  0.7103,
         -0.5414, -0.1094,  0.0379, -0.0954};                                  // 12개
    std::vector<float> vel_input{-0.1554,  0.0220, -0.9812,  0.4617,  3.8213,  0.6578, -2.4283, -1.7359,
         -1.2565,  1.2267,  0.7215, -0.6236};                                 // 12개
    std::vector<float> act_input{-0.1394, -0.0759,  0.2070, -0.2957, -0.2208, -1.7045,  3.5340,  2.0694,
         -2.2431,  0.0939,  0.0719,  0.1623};                                   // 12개

    std::vector<float> input_data;
    input_data.reserve(47);
    // input_data.insert(input_data.end(), clock_input.begin(), clock_input.end());
    // input_data.insert(input_data.end(), ang_vel_input.begin(), ang_vel_input.end());
    // input_data.insert(input_data.end(), grv_input.begin(), grv_input.end());
    // input_data.insert(input_data.end(), com_input.begin(), com_input.end());
    // input_data.insert(input_data.end(), pos_input.begin(), pos_input.end());
    // input_data.insert(input_data.end(), vel_input.begin(), vel_input.end());
    // input_data.insert(input_data.end(), act_input.begin(), act_input.end());

    input_data = {-0.9022, -0.4314,  0.2805,  0.2113,  0.0797,  0.1024, -0.1307, -1.0262,
         -0.8909, -0.2202, -0.1378, -0.0630, -0.0484,  0.0193,  0.0527, -0.1948,
         -0.5108,  0.8560,  0.7175, -0.5189, -0.1117,  0.0784, -0.0496, -1.1524,
          1.7270, -1.8848,  1.7471,  4.9528, -0.3233, -3.5787, -2.6640,  0.2626,
         -0.7586,  1.4138, -1.7517, -0.1426, -0.0670,  0.2043, -0.3038, -0.2298,
         -1.6998,  3.5266,  2.0630, -2.2437,  0.0994,  0.0630,  0.1696};

    torch::Tensor input_tensor = torch::from_blob(input_data.data(), {1, 47}, torch::kFloat32);

    print(input_tensor);

    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(input_tensor);

    // -------------------------------------------------------------------
    // 3. 모델 추론 실행
    // -------------------------------------------------------------------
    auto start = std::chrono::high_resolution_clock::now();

    at::Tensor output_tensor = module.forward(inputs).toTensor();

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    // -------------------------------------------------------------------
    // 4. 출력 결과 확인
    // -------------------------------------------------------------------
    float* output_data = output_tensor.data_ptr<float>();
    auto output_size = output_tensor.numel();

    std::cout << "\n--- Inference Output ---" << std::endl;
    for (int i = 0; i < output_size; ++i) {
        std::cout << "Action " << i << ": " << output_data[i] << std::endl;
    }

    std::cout << "\nExecution time: " << duration.count() << " microseconds" << std::endl;

    return 0;
}