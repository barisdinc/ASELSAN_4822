# ASELSAN 4822 , ASELSAN 4826 , ASELSAN 4926
Aselsan 48XX serisi cihazlara yeni özellikler kazandıran dönüşüm kiti yazılımı.


TODO List:
----------
- add antenna test step size and upper lower freq limits
- fix power on/offissues
- fix CTCSS tone signal
- add internal (onboard) eeprom usage
- fix memory structure for channels
- try to add packet radio AX25 and/or APRS
- add scan function
- add dual watch
- try to add squelch level control
- add end beep
- Add SWR alarm
- Power control using DAC 
- Add UHF Shift Function


ONEMLI GUNCELLEME :
------------------
PTT kontrolu icin TAMSAT Kit uzerindeki PTT cikisini ana kontrol karti uzerindeki J2'nin 3. pinine bagliyorduk. Ancak bu durumun dogru olmadigini farkettik ve J2'nin 3.pini yerine 8155 entegresini soktugumuz bacaklarindan 26 numarali bacaga lehimlemek gerekmektedir.
1 Agustoz 2019 tarihinden sonra hazirlanan HEX dosyalarini yuklediginizde, kartiniz uzerinde PTT out un bu 26 numarali bacaga baglanmis oldugu duruma gore calisacaktir, eski cihazlarda (yani bu modifikasyonun / duzenlemenin yapilmadigi cihazlarda PTT islevi ters calisacaktir)
Baglantinin nasil yapilacagi detayi icin guncellenmis olan modifikasyon dokumanindan inceleyiniz.



