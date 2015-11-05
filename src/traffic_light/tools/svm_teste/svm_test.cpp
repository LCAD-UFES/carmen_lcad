#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "dlib/svm.h"

using namespace std;
using namespace dlib;
using namespace cv;

int WIDTH, HEIGHT;

// The svm functions use column vectors to contain a lot of the data on which they
// operate. So the first thing we do here is declare a convenient typedef.  

// This typedef declares a matrix with 2 rows and 1 column.  It will be the object that
// contains each of our 2 dimensional samples.   (Note that if you wanted more than 2
// features in this vector you can simply change the 2 to something else.  Or if you
// don't know how many features you want until runtime then you can put a 0 here and
// use the matrix.set_size() member function)
typedef matrix<double, 200, 1> sample_type;

// This is a typedef for the type of kernel we are going to use in this example.  In
// this case I have selected the radial basis kernel that can operate on our 2D
// sample_type objects
typedef radial_basis_kernel<sample_type> kernel_type;


// Now we make objects to contain our samples and their respective labels.
static std::vector<sample_type> samples;
static std::vector<double> labels;

static sample_type sample;

void
read_images_red(string list);
void
read_images_green(string list);

void
read_image_test_red();
void
read_image_test_green();

void
read_image_test(string image);

int
main(int argc, char **argv)
{
    string list_red, list_green, list_test;

    if (argc != 6)
    {
        cerr << "Usage " << argv[0] << " list_red.txt list_green.txt list_test.txt width height" << endl;
        return EXIT_FAILURE;
    }

    list_red = argv[1];
    list_green = argv[2];
    list_test = argv[3];

    WIDTH = atoi(argv[4]);
    HEIGHT = atoi(argv[5]);

    read_images_red(list_red);
    read_images_green(list_green);

    cout << "Samples " << samples.size() << " Labels " << labels.size() << endl;

    for (uint i = 0; i < samples.size(); i++)
    {
        cout << "Samples " << samples.at(i)(0) << endl;
        cout << "Label " << labels.at(i) << endl;
    }


    // Here we normalize all the samples by subtracting their mean and dividing by their
    // standard deviation.  This is generally a good idea since it often heads off
    // numerical stability problems and also prevents one large feature from smothering
    // others.  Doing this doesn't matter much in this example so I'm just doing this here
    // so you can see an easy way to accomplish this with the library.  
    vector_normalizer<sample_type> normalizer;
    // let the normalizer learn the mean and standard deviation of the samples
    normalizer.train(samples);
    // now normalize each sample
    for (unsigned long i = 0; i < samples.size(); ++i)
        samples[i] = normalizer(samples[i]);


    // Now that we have some data we want to train on it.  However, there are two
    // parameters to the training.  These are the nu and gamma parameters.  Our choice for
    // these parameters will influence how good the resulting decision function is.  To
    // test how good a particular choice of these parameters is we can use the
    // cross_validate_trainer() function to perform n-fold cross validation on our training
    // data.  However, there is a problem with the way we have sampled our distribution
    // above.  The problem is that there is a definite ordering to the samples.  That is,
    // the first half of the samples look like they are from a different distribution than
    // the second half.  This would screw up the cross validation process but we can fix it
    // by randomizing the order of the samples with the following function call.
    randomize_samples(samples, labels);


    // The nu parameter has a maximum value that is dependent on the ratio of the +1 to -1
    // labels in the training data.  This function finds that value.
    const double max_nu = maximum_nu(labels);

    // here we make an instance of the svm_nu_trainer object that uses our kernel type.
    svm_nu_trainer<kernel_type> trainer;

    // Now we loop over some different nu and gamma values to see how good they are.  Note
    // that this is a very simple way to try out a few possible parameter choices.  You
    // should look at the model_selection_ex.cpp program for examples of more sophisticated
    // strategies for determining good parameter choices.
    cout << "doing cross validation" << endl;
    for (double gamma = 0.00001; gamma <= 1; gamma *= 5)
    {
        for (double nu = 0.00001; nu < max_nu; nu *= 5)
        {
            // tell the trainer the parameters we want to use
            trainer.set_kernel(kernel_type(gamma));
            trainer.set_nu(nu);

            cout << "gamma: " << gamma << "    nu: " << nu;
            // Print out the cross validation accuracy for 3-fold cross validation using
            // the current gamma and nu.  cross_validate_trainer() returns a row vector.
            // The first element of the vector is the fraction of +1 training examples
            // correctly classified and the second number is the fraction of -1 training
            // examples correctly classified.
            cout << "     cross validation accuracy: " << cross_validate_trainer(trainer, samples, labels, 3);
        }
    }


    // From looking at the output of the above loop it turns out that a good value for nu
    // and gamma for this problem is 0.15625 for both.  So that is what we will use.

    // Now we train on the full set of data and obtain the resulting decision function.  We
    // use the value of 0.15625 for nu and gamma.  The decision function will return values
    // >= 0 for samples it predicts are in the +1 class and numbers < 0 for samples it
    // predicts to be in the -1 class.
    trainer.set_kernel(kernel_type(0.00001));
    trainer.set_nu(0.03125);
    typedef decision_function<kernel_type> dec_funct_type;
    typedef normalized_function<dec_funct_type> funct_type;

    // Here we are making an instance of the normalized_function object.  This object
    // provides a convenient way to store the vector normalization information along with
    // the decision function we are going to learn.  
    funct_type learned_function;
    learned_function.normalizer = normalizer; // save normalization information
    learned_function.function = trainer.train(samples, labels); // perform the actual SVM training and save the results

    // print out the number of support vectors in the resulting decision function
    cout << "\nnumber of support vectors in our learned_function is "
            << learned_function.function.basis_vectors.size() << endl;

    // now lets try this decision_function on some samples we haven't seen before 

    int HIT = 0;
    int MISS = 0;
    int ENTIRE = 0;

    ifstream file;
    ofstream file_out;
    file_out.open("miss.txt");
    string line;
    file.open(list_test.c_str());

    if (file.is_open())
    {
        getline(file, line);
        while (!file.eof())
        {

            string s;
            std::vector<string> strings;
            istringstream iss(line);

            while (getline(iss, s, ' '))
            {
                //  cout << "Leitura: " << s << endl;
                strings.push_back(s);
            }
            read_image_test(strings[0]);
            //  cout << "The classifier output is " << learned_function(sample) << endl;
            if (learned_function(sample) > 0 && atoi(strings[1].c_str()) == 1)
            {
                HIT++;
            }
            else if (learned_function(sample) < 0 && atoi(strings[1].c_str()) == -1)
            {
                HIT++;
            }
            else
            {
                MISS++;
                file_out << strings[0] << endl;
                cv::Mat image;
                cv::Mat resized_image;

                image = imread("test/" + strings[0], 1);
                resized_image.create(HEIGHT, WIDTH, CV_8UC3);

                resize(image, resized_image, Size(WIDTH, HEIGHT), 0, 0, CV_INTER_CUBIC);

                std::vector<Mat> channels;
                split(resized_image, channels);

                Mat aux;
                aux = ((channels.at(2) - channels.at(1)) + 255) / 2;

                imwrite("miss/" + strings[0], aux);
            }

            ENTIRE++;

            getline(file, line);
        }
    }
    file_out.close();

    cout << "Size Images: " << ENTIRE << " HIT: " << HIT << " MISS: " << MISS << endl;

    ofstream fout("saved_function.dat", ios::binary);
    serialize(learned_function, fout);
    fout.close();
    //
    //    // now lets open that file back up and load the function object it contains
    ifstream fin("saved_function.dat", ios::binary);
    deserialize(learned_function, fin);

    //    // We can also train a decision function that reports a well conditioned probability
    //    // instead of just a number > 0 for the +1 class and < 0 for the -1 class.  An example
    //    // of doing that follows:
    //    typedef probabilistic_decision_function<kernel_type> probabilistic_funct_type;
    //    typedef normalized_function<probabilistic_funct_type> pfunct_type;
    //
    //    pfunct_type learned_pfunct;
    //    learned_pfunct.normalizer = normalizer;
    //    learned_pfunct.function = train_probabilistic_decision_function(trainer, samples, labels, 3);
    //    // Now we have a function that returns the probability that a given sample is of the +1 class.  
    //
    //    // print out the number of support vectors in the resulting decision function.  
    //    // (it should be the same as in the one above)
    //    cout << "\nnumber of support vectors in our learned_pfunct is "
    //            << learned_pfunct.function.decision_funct.basis_vectors.size() << endl;
    //
    //    read_image_test_green();
    //    cout << "This +1 class example should have high probability.  Its probability is: "
    //            << learned_pfunct(sample) << endl;
    //
    //    read_image_test_green();
    //    cout << "This -1 class example should have high probability.  Its probability is: "
    //            << learned_pfunct(sample) << endl;
    //
    //   
    //
    //
    //
    //    // Another thing that is worth knowing is that just about everything in dlib is
    //    // serializable.  So for example, you can save the learned_pfunct object to disk and
    //    // recall it later like so:
    //    ofstream fout("saved_function.dat", ios::binary);
    //    serialize(learned_pfunct, fout);
    //    fout.close();
    //
    //    // now lets open that file back up and load the function object it contains
    //    ifstream fin("saved_function.dat", ios::binary);
    //    deserialize(learned_pfunct, fin);
    //
    //    // Note that there is also an example program that comes with dlib called the
    //    // file_to_code_ex.cpp example.  It is a simple program that takes a file and outputs a
    //    // piece of C++ code that is able to fully reproduce the file's contents in the form of
    //    // a std::string object.  So you can use that along with the std::istringstream to save
    //    // learned decision functions inside your actual C++ code files if you want.  
    //
    //
    //
    //
    //    // Lastly, note that the decision functions we trained above involved well over 200
    //    // basis vectors.  Support vector machines in general tend to find decision functions
    //    // that involve a lot of basis vectors.  This is significant because the more basis
    //    // vectors in a decision function, the longer it takes to classify new examples.  So
    //    // dlib provides the ability to find an approximation to the normal output of a trainer
    //    // using fewer basis vectors.  
    //
    //    // Here we determine the cross validation accuracy when we approximate the output using
    //    // only 10 basis vectors.  To do this we use the reduced2() function.  It takes a
    //    // trainer object and the number of basis vectors to use and returns a new trainer
    //    // object that applies the necessary post processing during the creation of decision
    //    // function objects.
    //    cout << "\ncross validation accuracy with only 10 support vectors: "
    //            << cross_validate_trainer(reduced2(trainer, 10), samples, labels, 3);
    //
    //    // Lets print out the originacout << "X " << x * WIDTH + y << endl;l cross validation score too for comparison.
    //    cout << "cross validation accuracy with all the original support vectors: "
    //            << cross_validate_trainer(trainer, samples, labels, 3);
    //
    //    // When you run this program you should see that, for this problem, you can reduce the
    //    // number of basis vectors down to 10 without hurting the cross validation accuracy. 
    //
    //
    //    // To get the reduced decision function out we would just do this:
    //    learned_function.function = reduced2(trainer, 10).train(samples, labels);
    //    // And similarly for the probabilistic_decision_function: 
    //    learned_pfunct.function = train_probabilistic_decision_function(reduced2(trainer, 10), samples, labels, 3);
}

void
read_images_red(string list)
{
    ifstream file;
    string line;
    cv::Mat image;
    cv::Mat resized_image;

    file.open(list.c_str());

    if (file.is_open())
    {
        getline(file, line);
        while (!file.eof())
        {
            sample_type samp;
            image = imread("red/" + line, 1);

            resized_image.create(HEIGHT, WIDTH, CV_8UC3);

            resize(image, resized_image, Size(WIDTH, HEIGHT), 0, 0, CV_INTER_CUBIC);

            imwrite("bd/red/" + line, resized_image);

            std::vector<Mat> channels;
            split(resized_image, channels);

            Mat aux;
            aux = ((channels.at(2) - channels.at(1)) + 255) / 2;

            for (int x = 0; x < resized_image.rows; x++)
            {
                for (int y = 0; y < resized_image.cols; y++)
                {

                    samp(x * WIDTH + y) = (double) (aux.at<uchar>(x, y));
                }
            }
            samples.push_back(samp);
            labels.push_back(+1);

            getline(file, line);
        }
    }
}

void
read_images_green(string list)
{
    ifstream file;
    string line;
    cv::Mat image;
    cv::Mat resized_image;

    file.open(list.c_str());

    if (file.is_open())
    {
        getline(file, line);
        while (!file.eof())
        {
            sample_type samp;
            image = imread("green/" + line, 1);
            resized_image.create(HEIGHT, WIDTH, CV_8UC3);

            resize(image, resized_image, Size(WIDTH, HEIGHT), 0, 0, CV_INTER_CUBIC);

            imwrite("bd/green/" + line, resized_image);

            std::vector<Mat> channels;
            split(resized_image, channels);

            Mat aux;
            aux = ((channels.at(2) - channels.at(1)) + 255) / 2;


            for (int x = 0; x < resized_image.rows; x++)
            {
                for (int y = 0; y < resized_image.cols; y++)
                {

                    samp(x * WIDTH + y) = (double) (aux.at<uchar>(x, y));
                }
            }
            samples.push_back(samp);
            labels.push_back(-1);

            getline(file, line);
        }
    }
}

void
read_image_test_red()
{
    cv::Mat image;
    cv::Mat resized_image;

    image = imread("test/1.png", 1);
    resized_image.create(HEIGHT, WIDTH, CV_8UC3);

    resize(image, resized_image, Size(WIDTH, HEIGHT), 0, 0, CV_INTER_CUBIC);

    std::vector<Mat> channels;
    split(resized_image, channels);

    Mat aux;
    aux = ((channels.at(2) - channels.at(1)) + 255) / 2;

    for (int x = 0; x < resized_image.rows; x++)
    {
        for (int y = 0; y < resized_image.cols; y++)
        {
            sample(x * WIDTH + y) = (double) (aux.at<uchar>(x, y));
        }
    }
}

void
read_image_test_green()
{
    cv::Mat image;
    cv::Mat resized_image;

    image = imread("test/104.png", 1);
    resized_image.create(HEIGHT, WIDTH, CV_8UC3);

    resize(image, resized_image, Size(WIDTH, HEIGHT), 0, 0, CV_INTER_CUBIC);

    std::vector<Mat> channels;
    split(resized_image, channels);

    Mat aux;
    aux = ((channels.at(2) - channels.at(1)) + 255) / 2;

    for (int x = 0; x < resized_image.rows; x++)
    {
        for (int y = 0; y < resized_image.cols; y++)
        {
            sample(x * WIDTH + y) = (double) (aux.at<uchar>(x, y));
        }
    }
}

void
read_image_test(string image_name)
{
    cv::Mat image;
    cv::Mat resized_image;

    image = imread("test/" + image_name, 1);
    resized_image.create(HEIGHT, WIDTH, CV_8UC3);

    resize(image, resized_image, Size(WIDTH, HEIGHT), 0, 0, CV_INTER_CUBIC);
    imwrite("bd/test/" + image_name, resized_image);
    std::vector<Mat> channels;
    split(resized_image, channels);


    Mat aux;
    aux = ((channels.at(2) - channels.at(1)) + 255) / 2;


    for (int x = 0; x < resized_image.rows; x++)
    {
        for (int y = 0; y < resized_image.cols; y++)
        {
            sample(x * WIDTH + y) = (double) (aux.at<uchar>(x, y));
        }
    }
}
