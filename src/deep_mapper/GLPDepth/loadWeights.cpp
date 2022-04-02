#include <torch/script.h> // One-stop header.

#include <iostream>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

torch::jit::script::Module module;

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

int 
initialize_glpdepth()
{
  try
  {
    char *pPath;
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
    std::cerr << "GLPDepth: error loading the model\n";
    return -1;
  }
  return 1;
}


unsigned char* 
glpdepth_process_image(Mat img, int cut_param, int down_param)
{
  at::Tensor tensor;
  int new_w;
  int new_h;
  /* Transform image to tensor */
  try
  {
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
  }
  catch (const c10::Error &e)
  {
    std::cerr << "error loading mat\n";
    return NULL;
  }

  std::vector<torch::jit::IValue> inputs;
  try
  {
    // Create a vector of inputs.
    inputs.push_back(tensor);
  }
  catch (const c10::Error &e)
  {
    std::cerr << "error loading the inputs\n";
    return NULL;
  }
  // Execute the model and turn its output into a tensor.
  torch::NoGradGuard no_grad;
  c10::impl::GenericDict output = module.forward(inputs).toGenericDict();
  at::Tensor pred_d = output.at("pred_d").toTensor();
  pred_d = pred_d.squeeze();
  pred_d = pred_d * 256.0;
  pred_d = (pred_d / pred_d.max()) * 255;
  pred_d = pred_d.toType(torch::kU8);
  
  int width = pred_d.sizes()[0];
  int height = pred_d.sizes()[1];

  std::cout << pred_d << std::endl;

  for(unsigned int i = 0; i < cut_param; i++)
  {
    for(unsigned int j = 0; j < width; j++)
    {
      pred_d[i][j] = 1000;
    }
  }
  for(unsigned int i = height - down_param; i < height; i++)
  {
    for(unsigned int j = 0; j < width; j++)  
    {
      pred_d[i][j] = 0;
    }
  }

  
  cv::Mat color_mat;
  cv::Mat output_mat(cv::Size{ height, width }, CV_8U, pred_d.data_ptr<uchar>());
  applyColorMap(output_mat, color_mat, cv::COLORMAP_RAINBOW);
  
  cv::namedWindow("GLPDepth"); // Create a window
  cv::imshow("GLPDepth", color_mat); // Show our image inside the created window.
  cv::waitKey(0);

  return pred_d.data_ptr<uchar>();
}


int main(int argc, const char *argv[])
{

  
  
}
