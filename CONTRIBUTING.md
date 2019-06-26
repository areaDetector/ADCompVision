# Guide to contributing to ADCompVision

Author: Jakub Wlodek


Some pointers:  
* OpenCV uses Mat objects for images. They are written as a 'smart-pointer' class, meaning that they are passed
by-reference by default. Because of this, make sure to pass the image into the function with (Mat &img).
* The plugin works by calling functions in a helper library. If you wish to add an additional function for a more specific use case that the plugin does not support, follow the steps below.

## Adding a new CV function

There are two primary ways to add a new CV function to ADCompVision to customize the plugin. 

### Implementing user_function

The easier solution, and the one recommended if only one custom function is required, is to implement the user_function that has been integrated but left unimplemented. To implement it, open the NDPluginCVHelper.cpp file, and locate these two functions:

* user_function
* get_user_function_description

Implement both of these functions following the standards set by the other Wrapper functions, and you should be able to access your implementation from your EPICS client by selecting 'User Function' from Vision function set 3.

### Adding a brand new CV function

This is a more complicated process than simply implementing the user_function, and should be done only to add permanent new functions to the plugin.

To add a new CV function there are several files you will need to edit. First, in the NDCV.template file, find the record titled "CompVisionFunctionN", where N is an integer from 1 to 3. These are the three sets of supported functions, and generally follow the rule:
* Function set 1 contains basic OpenCV image processing functions
* Function set 2 contains more complex functions that are still general and with many use cases
* Function set 3 contains functions for specific use cases and custom implementations.  

Decide which of these sets your new function falls under, and add it to the Input and Output records.   


Next, in the `NDPluginCVHelper.h` file, change the `N_FUNC_#` value to take into account the new number of functions in the `CompVisionFunction#` PV you add your function to.  
Next, you will need to edit the `NDPluginCVHelper.cpp` and `NDPluginCVHelper.h` files. In `NDPluginCVHelper.h`, find the definition of `ADCVFunction_t` and add:
```
// Some basic flag types
typedef enum {
    ADCV_NoFunction         = 0,
    ADCV_GaussianBlur       = 1,
    ADCV_Threshold          = 2,
    .
    .
    .
    ADCV_YOURFUNCTION       = n,    <- Your function should be added in the appropriate location
    .
    .
    .
    ADCV_LastFunction       = 15,
} ADCVFunction_t;
```
Make sure to add your function type in the appropriate position, this is important for the conversion from PV value to function type. You will need to find which value it should be assigned. `ADCV_NoFunction` represents the first PV value in function set 1, and the rest count in order, going through set 1 -> set 2 -> set 3, disregarding the PVs for no function in set 2 and 3. Once you found the correct number add it to the enum, and fix any numbers that have changed. This will add the function to the list of possible functions handled by ADCompVision. Then, add the lines:

```
ADCVStatus_t YOURFUNCTION(Mat &img, double* inputs, double* outputs);
ADCVStatus_t get_YOURFUNCTION_description(string* inputDesc, string* outputDesc, string* description);
```
in the 'public' portion of the class declaration. You may follow the standard set by the other functions.   
Next, in the `NDPluginCVHelper.cpp` file, add your new function definitions. They should take the following form:

**The Wrapper**
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
        cvHelperStatus = "Image processed successfully with YOURFUNCTION"
    }catch(Exception &e){
        print_cv_error(e, functionName);
        status = cvHelperError;
    }

    return status;
}
```
**The I/O Description**
```
/**
 * Function that sets the I/O descriptions for YOURFUNCTION
 * 
 * @params[out]: inputDesc      -> array of input descriptions
 * @params[out]: outputDesc     -> array of output descriptions
 * @params[out]: description    -> overall function usage description
 * @return: void
 */
ADCVStatus_t NDPluginCVHelper::get_YOURFUNCTION_description(string* inputDesc, string* outputDesc, string* description){
    ADCVStatus_t status = cvHelperSuccess;
    int numInput = ?;
    int numOutput = ?;
    inputDesc[0] = "Input 1 Description";
    inputDesc[1] = "Input 2 Description";
    .
    .
    .
    outputDesc[0] = "Output 1 Description";
    outputDesc[1] = "Output 2 Description";
    .
    .
    .
    description = "Description of YOURFUNCTION";
    populate_remaining_descriptions(inputDesc, outputDesc, numInput, numOutput);
    return status
}
```
**NOTE** The commenting standard for these helper functions should be followed strictly, as it will allow for the provided python script to generate a manual for operation easily. Once you add the function and write the comment above it appropriately, you may run the `createIOmanual.py` script in the scripts/ directory with:
```
python3 createIOmanual.py
```
This will create a manual (in docs/manual.html) describing the inputs and outputs of each of the functions including your new custom function along with a description of each function as provided in the comments.  

Next, you must edit the `getFunctionDescription` function in `NDPluginCVHelper.cpp`. Add a case to the switch statement as follows:
```
case ADCV_YOURFUNCTION:
    status = get_YOURFUNCTION_description(inputDesc, outputDesc, description);
    break;
```
This will allow NDPluginCV to update descriptions for each input and output in real time when you select your function.

Finally, you need to edit the `processImage` function in `NDPluginCVHelper.cpp`. In the switch statement, add a case as follows:

```
case ADCV_YOURFUNCTION:
    status = YOURFUNCTION(image, inputs, outputs);
    break;
```
This will tell ADCompVision which function to process when your function is requested.

Your function should now be implemented into ADCompVision and is ready to be tested.