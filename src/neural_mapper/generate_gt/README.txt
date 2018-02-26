### Generating input dataset
This code generates the input dataset for neural mapper training.
The input dataset are six statistics from velodyne point cloud. Each statistic is a csv file:
	number of points
	max high
	min high
	mean high
	std high
	bean intensity
	File name format: <velodyne->timestamp>_<statistic_name>.csv
		ex: for number of points: <velodyne->timestamp>_num.csv
Each cell in the csv file, correspond to the value inside the cell with resolution R and Width(W) x High(H) defined in meters by command line parameter
The total resolution is WxHxR


Running
	create the folder where the dataset will be saved
		mkdir /dados/neural_mapper_dataset/
	compile the code:
		cd $CARMEN_HOME/src/neural_mapper/generate_gt/
		make
	Run any process that publish velodyne data
	Run the neural_mapper_dataset_generator
	./neural_mapper_dataset_generator W H R Skip_clouds
	 
