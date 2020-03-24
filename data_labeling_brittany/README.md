# Data Labeling

This package allow label the data to train the neural network. It creates for each LIDAR scan one black and white image. Where the white points are the LIDAR points where was a person. After, this images have been concatenate with different configurations to represent the walking of the people.

The following configurations have been created to get the best neural network model: 

* concat_5_1: concatenating 5 matrices in a row, which would represent 5 consecutive laser scans [n, n + 1, ..., n + 4].
* concat_5_2: by concatenating 5 images, selecting from the list of available matrices one matrix out of each two [n, n + 2, ..., n + 8].
* concat_5_3: by concatenating 5 images, one matrix out of 3 is selected from the list of matrices [n, n + 3, ..., n + 12].
* concat_10_1: concatenating 10 matrices in a row, which would represent 10 consecutive laser scans [n, n + 1, ..., n + 9].
* concat_10_2: by concatenating 10 images, selecting from the list of available matrices one matrix out of each two [n, n + 2, ..., n + 18].
* concat_10_3: by concatenating 10 images, one matrix out of 3 is selected from the list of matrices [n, n + 3, ..., n + 27].

## Tree of directories
* /rosbags  
* /npy_global_files  
	* /npy_train  
	* /npy_label  

## Steps

1.- To create the images execute script generate_npys.sh:  
```
$ cd data_labeling_brittany/scripts/  
$ ./generate_npys.sh <absolute_path_rosbags> <absolute_path_where_save_npys>  
```

2.- Divide npys in two directories, one for train another for test.  

3.- Create the class for each npy file, execute create_npy_class.py:  
```
$ cd data_labeling_brittany/scripts/  
$ python create_npy_class.py <absolute_path_npy_files_train>  
$ python create_npy_class.py <absolute_path_npy_files_test>  
```
4.- Join data to train the neural network model, execute join_data_to_train.py:  
```
$ cd data_labeling_brittany/scripts/  
$ python join_data_to_train.py <absolute_path_npy_global_directory>  
```




