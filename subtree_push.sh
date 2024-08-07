# Usage:
# Enter the root directory folder (.git directory) and commit any changes including the libraries that you intent to change
# After committing changes, run the bash script from the root directory of the git repository. It will push changes 
# to all of the below repositiories (stored as subtrees in the /libs folder)
git fetch git@github.com:Hover-Disaster/STM32_BMX055.git
git fetch git@github.com:Hover-Disaster/STM32_MS5611.git
git fetch git@github.com:Hover-Disaster/STM32_SD.git 
git fetch git@github.com:Hover-Disaster/STM32_LoRa.git 
git fetch git@github.com:Hover-Disaster/Strelka_State_Machine.git
git fetch git@github.com:s-park21/CHIRP.git
git fetch git@github.com:Hover-Disaster/STM32_EKF.git
git fetch git@github.com:Hover-Disaster/STM32_ADXL375.git
git fetch git@github.com:Hover-Disaster/STM32_EKF_Full.git

git subtree push --prefix=libs/STM32_BMX055 git@github.com:Hover-Disaster/STM32_BMX055.git main
git subtree push --prefix=libs/STM32_MS5611 git@github.com:Hover-Disaster/STM32_MS5611.git main
git subtree push --prefix=libs/STM32_SD git@github.com:Hover-Disaster/STM32_SD.git main
git subtree push --prefix=libs/STM32_LoRa git@github.com:Hover-Disaster/STM32_LoRa.git main
git subtree push --prefix=libs/state_machine git@github.com:Hover-Disaster/Strelka_State_Machine.git main
git subtree push --prefix=libs/CHIRP git@github.com:s-park21/CHIRP.git main
git subtree push --prefix=libs/STM32_EKF git@github.com:Hover-Disaster/STM32_EKF.git main
git subtree push --prefix=libs/STM32_ADXL375 git@github.com:Hover-Disaster/STM32_ADXL375.git main
git subtree push --prefix=libs/STM32_EKF_Full git@github.com:Hover-Disaster/STM32_EKF_Full.git main
