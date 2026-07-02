# Kontrola stanja sigurnosnih pojaseva u automobilu

## Opis projekta

Cilj projekta je realizacija softvera za kontrolu stanja sigurnosnih pojaseva u automobilu. Sistem prati da li su vozač i suvozač vezani, kao i da li je suvozač prisutan na sjedištu. Na osnovu dobijenih podataka sistem prikazuje stanje na 7-segmentnom displeju, upravlja LED barom i šalje odgovarajuće poruke preko serijske komunikacije. Podaci sa senzora simuliraju se pomoću UniCom kanala 0, komande i status sistema koriste UniCom kanal 1, dok se upozorenja šalju na UniCom kanal 2.

## Zahtjevi projekta

* minimum 4 taska:

  * task za prijem podataka od senzora
  * task za slanje i prijem podataka od PC-ja
  * task za obradu podataka
  * task za prikaz na displeju

* taskovi za prijem i slanje podataka treba da budu jednostavni i da podatke prosleđuju taskovima za obradu

* sinhronizaciju između taskova realizovati pomoću semafora ili mutexa

* podatke između taskova slati pomoću redova, odnosno queue mehanizma

* senzore simulirati preko UniCom kanala 0, uz automatski odgovor pomoću trigger-a na svakih 200 ms

* pratiti stanje pojasa vozača, stanje pojasa suvozača i pritisak na sjedištu suvozača

* komunikaciju sa PC-jem realizovati preko UniCom kanala 1 i 2

* podržane komande su START, STOP i PRAG_broj

* ako je sistem uključen, a neki od potrebnih pojaseva nije vezan, slati upozorenje na UniCom kanal 2

* koristiti LED bar sa jednim ulaznim i dva izlazna stubca

* uključivanjem donje LED diode ulaznog stubca uključuje se sistem

* ako je sistem uključen, donja LED dioda prvog izlaznog stubca treba da svijetli

* ako postoji alarm, drugi izlazni stubac treba da blinkuje

* na 7-segmentnom displeju prikazati stanje pojasa vozača, stanje pojasa suvozača i prisustvo suvozača, sa praznim segmentom između podataka

* kod treba da poštuje MISRA pravila gdje je to moguće

## Pokretanje programa

* Preuzeti ceo repozitorijum sa GitHub-a.

* Otvoriti projekat u Visual Studio okruženju.

* Otvoriti solution fajl:

  * FreeRTOS_simulator_final.sln

* U okviru projekta otvoriti fajl:

  * main_application.c

* U gornjem dijelu Visual Studio okruženja podesiti platformu na x86.

* Pokrenuti program pomoću opcije:

  * Local Windows Debugger

## Pokretanje periferija

* Preko Command Prompt-a ući u folder sa periferijama.

* Pokrenuti 7-segmentni displej sa 5 cifara:
  
  * Seg7_Mux.exe 5

* Pokrenuti LED bar sa jednim ulaznim i dva izlazna stubca:
  
  * LED_bars.exe gRR

* Pokrenuti tri UniCom terminala:
  
  * UniCom.exe 0
  * UniCom.exe 1
  * UniCom.exe 2

* UniCom kanal 0 koristi se za simulaciju senzora, UniCom kanal 1 za komande i status sistema, a UniCom kanal 2 za upozorenja. Važno je da se za svaki UniCom kanal pokrene poseban Command Prompt.

## Podešavanje UniCom kanala 0

Na UniCom kanalu 0 potrebno je uključiti automatski odgovor. U polje za trigger unosi se:

  * T

U polje za automatski odgovor unosi se poruka u obliku:

  * $V,S,P#


gdje V predstavlja stanje pojasa vozača, S stanje pojasa suvozača, a P vrijednost senzora pritiska na sjedištu suvozača. Nakon unosa poruke potrebno je uključiti Auto opciju i kliknuti na ok1.

## Komande na UniCom kanalu 1

Na UniCom kanalu 1 mogu se slati sledeće komande:

  * START
  * STOP
  * PRAG_broj

Komanda START uključuje sistem, STOP isključuje sistem, dok PRAG_broj postavlja prag pritiska za detekciju prisustva suvozača.Ako se komanda šalje preko SEND CODE polja, završava se karakterom CR(\0d). Nakon ispravne komande sistem vraća poruku OK.

## Testiranje softvera

* Za test uključenog sistema poslati komandu START ili uključiti donju LED diodu na zelenom ulaznom stubcu. Očekuje se da donja LED dioda prvog crvenog stubca svijetli.

* Za test oba vezana pojasa na UniCom0 postaviti:

  * $1,1,000#

Očekivano je da nema alarma, a na displeju se prikazuje stanje F _ F _ 0.

* Za test nevezanog vozača postaviti:

  * $0,1,000#

Očekuje se alarm za vozača i blinkanje drugog crvenog stubca.

* Za test prisutnog i nevezanog suvozača postaviti:

  * $1,0,700#

Ako je prag 500, očekuje se alarm za suvozača.

* Za test kada vozač i suvozač nisu vezani postaviti:

  * $0,0,700#

Očekuje se alarm za oba pojasa.

* Za test promjene praga poslati komandu:

  * PRAG_800\0d

Ako je poslata senzorska vrijednost $1,0,700#, suvozač se tada ne detektuje jer je pritisak manji od zadatog praga.

* Za isključivanje sistema poslati komandu:

  * STOP\0d


Očekuje se gašenje izlaza i prestanak slanja alarma.

## Autori

Ime i prezime: 
 * Aleksandar Trifunović EE 21/2021
 * Filip Lazić EE 144/2021
Predmet: Autoelektronika
Projekat: Kontrola stanja sigurnosnih pojaseva u automobilu
