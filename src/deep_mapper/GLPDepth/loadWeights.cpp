#include <torch/script.h> // One-stop header.

#include <iostream>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

at::Tensor matToTensor(cv::Mat frame, int h, int w, int c)
{
  cv::cvtColor(frame, frame, CV_BGR2RGB);
  frame.convertTo(frame, CV_32FC3, 1.0f / 255.0f);
  auto input_tensor = torch::from_blob(frame.data, {1, h, w, c});
  input_tensor = input_tensor.permute({0, 3, 1, 2});

  torch::DeviceType device_type = torch::kCPU;
  //    if (torch::cuda::is_available()) {
  // device_type = torch::kCUDA;
  //    }
  input_tensor = input_tensor.to(device_type);
  return input_tensor;
}

cv::Mat tensorToOpenCv(at::Tensor out_tensor, int h, int w, int c)
{
  out_tensor = out_tensor.squeeze().detach().permute({1, 2, 0});
  out_tensor = out_tensor.mul(255).clamp(0, 255).to(torch::kU8);
  out_tensor = out_tensor.to(torch::kCPU);
  cv::Mat resultImg(h, w, CV_8UC3);
  // cv::Mat resultImg(h, w, CV_8UC1);
  std::memcpy((void *)resultImg.data, out_tensor.data_ptr(), sizeof(torch::kU8) * out_tensor.numel());
  return resultImg;
}

int main(int argc, const char *argv[])
{

  torch::jit::script::Module module;
  char *pPath;
  try
  {

    char *glpdepthPath;
    pPath = getenv("CARMEN_HOME");
    glpdepthPath = (char *)"/src/deep_mapper/GLPDepth/ckpt/best_model_kitti.pt";
    char *path = (char *)malloc(1 + strlen(pPath) + strlen(glpdepthPath));
    strcat(path, pPath);
    strcat(path, glpdepthPath);

    // Deserialize the ScriptModule from a file using torch::jit::load().
    module = torch::jit::load(path);
    module.eval();
    module.to(at::kCPU);
  }
  catch (const c10::Error &e)
  {
    std::cerr << "error loading the model\n";
    return -1;
  }
  at::Tensor tensor;
  int new_w;
  int new_h;
  try
  {

    std::string image_path = "/src/deep_mapper/GLPDepth/input/000005.png";
    std::stringstream ss;
    ss << pPath << image_path.c_str();
    Mat img = imread(ss.str(), IMREAD_COLOR);

    cv::Mat image_resized;
    int w = img.cols;
    int h = img.rows;
    new_h = round(h / 32) * 32;
    new_w = round(w / 32) * 32;

    cv::resize(img, image_resized, cv::Size(new_w, new_h));
    cv::Mat image_resized_float;
    image_resized.convertTo(image_resized_float, CV_32F, 1.0 / 255);
    tensor = matToTensor(image_resized_float, new_h, new_w, 3);
    tensor = tensor.contiguous();

    // std::cout << tensor << std::endl;
  }
  catch (const c10::Error &e)
  {
    std::cerr << "error loading mat\n";
    return -1;
  }
  std::vector<torch::jit::IValue> inputs;
  try
  {
    // Create a vector of inputs.
    inputs.push_back(tensor);

    // Execute the model and turn its output into a tensor.
  }
  catch (const c10::Error &e)
  {
    std::cerr << "error loading the inputs\n";
    return -1;
  }
  // at::Tensor output;

  torch::NoGradGuard no_grad;
  auto output = module.forward(inputs).toGenericDict();
  auto pred_d = output.at("pred_d");
  //auto TensorList = output.toTensor();
  //std::cout << TensorList << std::endl;
  // at::Tensor new_output = output[0].toTensor();
  // new_output = new_output.contiguous();
  std::cout << pred_d << std::endl;

  // cv::Mat image = tensorToOpenCv(new_output, new_h, new_w, 1);

  // cv::namedWindow("Profundidade"); // Create a window

  // cv::imshow("Profundidade", image); // Show our image inside the created window.

  // cv::waitKey(0);

  std::cout << "ok\n";
}
