# Guide to contributing to ADCompVision


Some pointers:  
* OpenCV uses Mat objects for images. They are written as a 'smart-pointer' class, meaning that they are passed
by-reference by default. Because of this, make sure to pass the image into the function with (Mat &img).
* The plugin works by calling functions in a helper library. If you wish to add an additional function for a more specific use case that the plugin does not support, follow the steps below.

# Adding a new CV function

To add a new CV function there are several files you will need to edit. First, in the NDCV.template file, find the records that store the CV functions. You will need to add your function to one of these PVs. Follow the rules for EPICS multi-bit-binary-outputs/inputs to add your function to the PV.   
Next, you will need to edit the NDPluginCVHelper.cpp and NDPluginCVHelper.h files. In NDPluginCVHelper.h, find the definition of ADCVFunction_t and add:
```
// Some basic flag types
typedef enum {
    ADCV_NoFunction         = 0,
    ADCV_EdgeDetectionCanny = 1,
    ADCV_Threshold          = 2,
    ADCV_YOURFUNCTION       = n,
} ADCVFunction_t;
```
where n is the next integer. This will add the function to the list of possible functions handled by ADCompVision. Then, add the line:

```
ADCVStatus_t YOURFUNCTION(Mat &img, double* inputs, double* outputs);
```
in the 'public' portion of the class declaration. You may follow the standard set by the other functions.   
Next, in the NDPluginCVHelper.cpp file, add your new function definition. it should take the following form:
```
/**
 * WRAPPER  ->  YOURFUNCTIONNAME
 * YOUR_FUNCTION_DESCRIPTION
 *
 * @inCount     -> n
 * @inFormat    -> [Param1 (Int), Param2 (Double) ...]
 *
 * @outCount    -> n
 * @outFormat   -> [Param1 (Int), Param2 (Double) ...]
 */
ADCVStatus_t NDPluginCVHelper::YOURFUNCTION(Mat &img, double* inputs, double* outputs){
    const char* functionName = "YOURFUNCTION";
    ADCVStatus_t status = cvHelperSuccess;
    param1 = inputs[0];
    param2 = inputs[1];
    .
    .
    .

    try{
        // Process your image here
        // Don't make copies, pass img, img as input and output to OpenCV.
        // Set output values with output[n] = value. cast non-double values to double
        // If you need more inputs or outputs, add more PVs following previous examples.
    }catch(Exception &e){
        print_cv_error(e, functionName);
        status = cvHelperError;
    }

    return status;
}
```
**NOTE** The commenting standard for these helper functions should be followed strictly, as it will allow for the provided python script to generate a manual for operation easily. Once you add the function and write the comment above it appropriately, you may run the createIOmanual.py script in the docs/ directory with:
```
python3 createIOmanual.py
```
This will create a manual describing the inputs and outputs of each of the functions including your new custom function along with a description of each function as provided in the comments.  
Finally, you need to edit the 'processImage' function in NDPluginCVHelper.cpp. In the switch statement, add a case as follows:

```
case ADCV_YOURFUNCTION:
    status = YOURFUNCTION(image, inputs, outputs);
    break;
```

Your function should now be implemented into ADCompVision and is ready to be tested.