
mergehex.exe --merge nrf52832_xxaa.hex s132_nrf52_3.0.0_softdevice.hex --output production_final1.hex 

nrfjprog -f NRF52 --eraseall


nrfjprog -f NRF52 --program "production_final1.hex" --verify


nrfjprog -f NRF52 --reset

del production_final1.hex

pause
