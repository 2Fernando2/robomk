Annex 2:  DNN model to recognise numbers on the rooms’ walls

We will create a NEW separate component to process the camera images and return a 0-9 digit or no-digit. This will be created using robocompdsl and setting the language to Python. You need to:
- move your existing localiser code in the actividad 4 folder to a sub-folder named localiser. Do this with git mv after cleaning the current local version.
- Copy mnist_detector  from beta_robotica_class to Activity4
- Create an IDSL file under ~/robocomp/interfaces/IDSLs/ named MNIST.idsl specifying a data structure and a method that will be called from within your localiser component, and that will be implemented in the new dnn_processor component.
- Create the new component using robocompdsl mnist-detector.cdsl ., edit it to adde the Camera360RGB.idsl and MNist.idsl interfaces and then do robocompdsl dnn_processor.cdsl  to generate the Python code.
- Open in Pycharm-professional or another IDE
- The new component has to:
- - load the trained model (my_network.pt) once
- - In the process_image() method (defined in the MNist.idsl interface and created by robocompdsl in the specificworker.py file)
- - - read the image from WebotsBridge
- - - pass it to the model
- - - return the result
- In your localiser component (C++) edit the .cdsl file to add the new interface MNist.idsl and regenerate it. You will get an additional proxy to remotely access 
the new Python component.  Set the ports properly.


This Python program downloads the dataset and creates a mosaic visualization of a sample of the 10K images.
You need another Python program to train the dataset and create a my_network.pt file with the network architecture and weights. Specify a 2-layer convolutional net with matching sizes for the input layer (image size) and the output layer (10 elements coding the digits).
Before integrating it in the new component, create another script to test the model with the test images. It should give you a success rate of over 95%.
In beta-robotica-class you can find a new Python component, mnist_detector,  that requires the image from the robot’s camera through WebotsBridge. Complete the code in compute() to:
1. load the trained DNN model
2. search for a candidate number region, i.e. a black square.
3. if found, do a forward pass on the detected region.
When the get_number() method is called from the localiser component, it will return the current detection number, if found, or -1 otherwise
