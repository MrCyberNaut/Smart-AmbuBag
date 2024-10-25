# Smart-AmbuBag
The Smart Ventilator System is an embedded project designed to assist in the respiratory support of patients based on their age group—infants, teens, and adults. Utilizing an ESP32 microcontroller, this system integrates a servo motor to drive an ambu bag, adjusting the breaths per minute according to the specified guidelines 


Key features of the system include:

Heart Rate Monitoring: Utilizes the MAX30100 pulse oximeter sensor to continuously monitor the heart rate. An audible alert (buzzer) is triggered if the heart rate falls below critical thresholds (100 BPM for infants, 75 BPM for teens, and 60 BPM for adults).
Environmental Sensing: Incorporates a DHT11 sensor to measure temperature and humidity, providing additional context for patient care.
User Interface: Displays the current heart rate and selected settings on an LCD screen, enabling easy monitoring and adjustments.
User Controls: A button allows users to cycle through settings for different age groups, making the device adaptable to various patient needs.

This project serves as an educational tool and a prototype for a smart medical device, emphasizing the integration of multiple sensors and components to enhance patient care in respiratory distress scenarios. The code and hardware designs are included for those interested in building or improving upon the system.
