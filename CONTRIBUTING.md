# Guide to contributing to ADCompVision


Some pointers:  
* OpenCV uses Mat objects for images. They are written as a 'smart-pointer' class, meaning that they are passed
by-reference by default. Because of this, make sure to pass the image into the function with (Mat &img).
* The plugin works by calling functions in a helper library. Eventually, a 'custom-function' option will be added so that a custom use case can be easily added and supported.