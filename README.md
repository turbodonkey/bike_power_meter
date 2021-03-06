# Upgrade your spin bike with an ANT+ Power Meter (PWR) and Speed (SPD) sensor

This is a small project based on the [sparkfun nrf52480 mini](https://www.sparkfun.com/products/15025) which is based on [Nordic Semiconductor's nRF52480 chip](https://www.nordicsemi.com/?sc_itemid=%7B2DC10BA5-A76E-40F8-836E-E2FC65803A71%7D). This project can be used to implement any bike which provides a digital hall effect trigger for cadence and analogue signal for resistance / gear. Based on these inputs an ANT+ PWR and ANT+ SPD sensor can be developed using the awesome nRF52480 chip.

Also note, this project will require a programmer to program the Softdevice. To the best of my knowledge (and attempts) you cannot program the s340 softdevice without a programmer. I used the cheap [educational segger mini](https://www.segger.com/products/debug-probes/j-link/models/j-link-edu-mini/) if you are looking for a recommendation.

As ANT+ requires a license and registration as a developer, this project does not provide two crucial elements to make this successful;
1. You need the Softdevice s340 to operate ANT on the nRF52480. This is ONLY available through [thisisant.com](thisisant.com).
2. You need the ANT+ Network Num and set it in the project under #define ANTPLUS_NETWORK_NUM.
Again this is ONLY available through registration and developer license agreement at [thisisant.com](thisisant.com). They are freely available behind the registration and license acceptance process. Note: You cannot distrubute these, and can only be used for educational, not-for-profit projects... exactly like this one.

To map the Speed in m/s to the Power rating I used a simple quadratic that mirrored some [engineering work here](https://www.gribble.org/cycling/power_v_speed.html). Improvements need to be made here I suspect. I will link the function to desmos in another update.

Calibration of the PWR curve is a hack at best until I get access to something better to qualify it. The calibration and auto-calibration feature of ANT+ PWR is not implemented, so this is another area of work. However I have set a table to reflect ADC values against PWR based on the bike I used, an older Keiser M3 spin bike (which my model is not for sale anymore but latest models are here - not even the [latest models](https://www.keiser.com/fitness-equipment/cardio-training/m3i-indoor-cycle) talk ANT+ natively.

The Keiser M3 has a [circuit board that sends the signals](https://www.sportsmith.net/itemform.aspx?item=46112&ReturnUrl=/productsearch.aspx?dsNav=Ro%3a12%2cN%3a751349&dsDimSearch=Dxp%3a5), both cadence and resistance (which is a potentiometer), to the bike computer . For my use I simply tapped the 4 wire connection for input into the sparkfun nrf52480 mini. Specifically hall effect digital pin went to pin 23 (NRF_GPIO_PIN_MAP(0, 23)) and after supplying 3V3 and GND to the pot the wiper went to analog pin 7 (NRF_SAADC_INPUT_AIN7) which I think is pin 31. The sparkfun nrf52480 mini comes with a board header file which maps all the pins to common #defines used in the SDK. I've edited it a little but you can [follow sparkfuns guide to set it up in your environment](https://learn.sparkfun.com/tutorials/nrf52840-advanced-development-with-the-nrf5-sdk) - note this project only caters for the armgcc build toolchain. Its free and most familiar to me and easy to access on OS X.

The project utilises the [latest nRF5 SDK](https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.sdk5.v15.3.0/index.html) which at the time of authoring is 15.3.0 

Finally this project has served my purpose very well, it registers both the Power and Cadence as an ANT+ PWR profile (Channel 0) and also provides Speed as an ANT+ SPD profile (Channel 1). This coupled with a ANT+ HRM (outside the scope of this project) and I was able to register it all on my [Garmin Forerunner 935 watch](https://buy.garmin.com/en-AU/AU/p/564291). All whilst being able to Zwift at the same time using my computer with ANT+ USB dongle. Note: iOS and Android devices are not supported at this stage as I have not implemented the Bluetooth stack.

I upload the work in the hope that others will share and extend this work mainly in these areas I'd like to encourage;
1. Enhancements to calibration (auto-calibration functionality) and quality of authentic PWR and SPD values.
2. Leverage Bluetooth stack - its available in the Softdevice s340 but I have built strickly ANT+. This will help with native sync to iOS and Android apps and also allow for functional project sharing out of the box without ANT+ license restrictions.
3. The Ultimate Goal - ANT+ FE-C. This is a computer controlled spec and will require the project to implement a stepper motor to increment and decrement resistance requests based on connected computer. I consider this to be the ultimate end game which the community can use to implement a FE-C stack and upgrade many old spin bikes to FE-C.

Cheers.
