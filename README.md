# BMES-CPM

HX711_ADC Library Documentation
```
Initialization:
 begin(): Initializes the communication with the HX711 and sets the gain to 128 (default).
 begin(uint8_t gain): Initializes the communication with the HX711 and sets the gain (32, 64, or 128).
 Tare (Zero Point Calibration):
 tare(): Performs tare operation (blocking, waits until finished).
 tareNoDelay(): Initiates tare operation (non-blocking, continues in background).
 getTareStatus(): Returns true if the tare operation is complete.

Data Acquisition:
 update(): Reads a new weight sample (blocking, waits for conversion).
 dataWaitingAsync(): Checks if new weight data is available (non-blocking).
 updateAsync(): Reads new weight data if available (non-blocking, called after dataWaitingAsync).
 getData(): Returns the latest weight value (after applying calibration and filtering).

Calibration:
 setCalFactor(float cal): Sets the calibration factor for weight conversion (weight = raw data / calFactor).
 getCalFactor(): Returns the current calibration factor.
 getNewCalibration(float known_mass): Calculates and sets a new calibration factor based on a known mass.

Other Functions:
 setSamplesInUse(int samples): Sets the number of samples used for averaging and filtering (rounded down).
 getSamplesInUse(): Returns the current number of samples in use.
 resetSamplesIndex(): Resets the index for the dataset.
 refreshDataSet(): Refreshes the entire dataset with new conversions (blocking).
 getDataSetStatus(): Checks if the dataset is filled with conversions.
 getSignalTimeoutFlag(): Indicates if the HX711 communication timed out.
 setReverseOutput(): Reverses the output value (positive/negative).
 getTareOffset(): Gets the current tare offset (raw data value).
 setTareOffset(long newoffset): Sets a new tare offset (raw data value).
 powerUp(): Powers up the HX711 chip.
 powerDown(): Powers down the HX711 chip.
 getReadIndex(): Returns the current dataset read index (debugging).
 getConversionTime(): Returns the latest conversion time in milliseconds (debugging).
 getSPS(): Estimates the HX711 conversions per second (debugging).
 getTareTimeoutFlag(): Returns the tare operation timeout flag (debugging).
 disableTareTimeout(): Disables the tare operation timeout.
 getSettlingTime(): Calculates the estimated settling time based on conversion time and sample count (debugging).
```
