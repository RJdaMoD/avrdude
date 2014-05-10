avrdude
=======

avrdude with a Linux SPI programmer type

Kevin Cuzner Jun 2013
kevin@kevincuzner.com

Using baud-rate to control SPI frequency

Rui Azevedo (neu-rah) Jun 2013
ruihfazevedo[arroba]gmail.com

additional SPI programmer using TI Stellaris Launchpad with stellarisSPI
( see https://github.com/RJdaMoD/stellarisSPI )
uses pins PA2(SCLK), PA4(MISO), PA5(MOSI) and PFx(RESET)
where x is 1 or 2, configured in avrdude.conf (flashes red/green led when high)
spi speed can be set via bitclock parameter and defaults to 115200.

Roger John (RJdaMoD) May 2014
roger.john@rub.de

