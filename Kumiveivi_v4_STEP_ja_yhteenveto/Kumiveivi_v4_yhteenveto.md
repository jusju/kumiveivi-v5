# Kumiveivi 1:20 — uusimman löytyneen version vientiyhteenveto

Vienti tehty 14.10.2026. Lähtötiedosto: rubber_winder_1to20_v4_fixed.zip, tallennettu 8.10.2026. Geometria on kopioitu alkuperäisistä v4-STEP-tiedostoista muuttamatta sitä. Tämä ei ole uusi suunnitteluversio.

## Vientitiedostot

- **Kumiveivi_v4_vaihteisto.step:** U-runko ja neljä rataskappaletta kokoonpanoasennossa. Ei akseleita, kiinnittimiä, veivin vartta, kahvaholkkia tai ulostulokytkintä.
- **Kumiveivi_v4_elektroniikan_tilavaraukset.step:** sama vaihteisto sekä akun, Nanon, mikrokytkimen ja näytön yksinkertaiset tilavarauskappaleet. Tilavaraukset eivät ole elektroniikan valmistusmalleja tai valmiita kiinnikkeitä.
- ZIP-paketin **osat_STEP/** sisältää kaikki kahdeksan erillistä valmistettavaa osaa, myös veivin varren, kahvaholkin ja valinnaisen ulostulokytkimen. Osat ovat omissa paikallisissa koordinaatistoissaan.
- **lahdetiedot/** sisältää alkuperäiset parametriluvut, v4-muutoskuvauksen ja aiemman tarkistusraportin. Parametrien yleiset hammastusluvut ovat osittain v3-perua: v4_modifications-osion poikkeukset ovat ensisijaisia osille 03 ja 04.

STEP sisältää CAD-geometrian, mutta ei alkuperäistä parametrista piirrehistoriaa, kokoonpanon liikerajoitteita tai toimivaa välityssimulaatiota. Millimetrit. Avaa/tua tiedosto CAD-ohjelmassa; älä muuta mittakaavaa.

## Päämitat

| Ominaisuus | Arvo |
|---|---:|
| Nopeussuhde, sisään → ulos | 1 → 20 |
| Akselien lukumäärä | 4 |
| Hammasvälityksiä | 3 |
| Rungon ulkomitat | 111,5 × 84,5 × 38,4 mm |
| Rungon sivuseinämien väli | 28,8 mm |
| Sivuseinämän paksuus | 4,8 mm |
| Suunnittelun skaalauskerroin | 1,6 |
| Hammasmoduuli | 1,28 mm |
| Rattaan hammaskehän leveys | 6,4 mm |
| Akseli- ja veivinreiät | nimellinen Ø5,0 mm |
| Veivin varsi | 128 × 24 × 8 mm |
| Kahvaholkki | pituus 32; ulko-Ø13,6; sisä-Ø5 mm |
| Valinnainen ulostulokytkin | ulko-Ø19,2; pituus 19,2; keski- ja poikittaisreikä Ø5 mm |
| Laskuritilan leveys | noin 32,5 mm |

Yksi veivin kierros tuottaa 20 ulostuloakselin kierrosta. Välitys kasvattaa nopeutta; se ei kasvata ulostulon vääntömomenttia.

## Ratasketju ja v4-muutokset

30T → 12T / 32T → 12T / 36T → 12T.

Kokonaisvälitys = (30/12) × (32/12) × (36/12) = 20.

| Osa | Hampaat | Ulkohalkaisija, v4 |
|---|---|---|
| 02, tuloratas | 30 | 41,344 mm |
| 03, yhdistelmäratas | 12 + 32 | 17,792 + 43,520 mm |
| 04, yhdistelmäratas | 12 + 36 | 17,792 + 49,024 mm |
| 05, ulostuloratas | 12 | 18,304 mm |

Osien 03 ja 04 pikkurattaiden hampaan kärjen leveydeksi ilmoitetaan v4-lähteessä noin 0,76 mm (aiemmin noin 0,28 mm). Niiden addendum on 0,95 × moduuli ja hammastuksen backlash-parametri 0,10 mm. Osan 03 32T-kehän addendum on 1,00 × moduuli; sen yläpinta on rakennettu tasaiseksi. Ulostulon 12T-ratas on eri geometriaa kuin yhdistelmärattaiden korjatut 12T-kehät.

Rungon muodot, akselien paikat, reikäkoot ja 20-kertainen nopeussuhde säilyivät v4:ssä. Erilliset +8 mm halkaisijan koerattaat on jätetty vientipaketista pois: lähde merkitsee ne toimivan ratasketjun kanssa yhteensopimattomiksi.

## Akselien paikat

X,Y millimetreinä rungon alkuperäisessä koordinaatistossa:

| Akseli | X | Y |
|---|---:|---:|
| 1: 30T, veivin tulo | 25,072 | 29,872 |
| 2: 12T + 32T | 52,242 | 31,156 |
| 3: 12T + 36T | 37,559 | 55,559 |
| 4: 12T, ulostulo | 65,056 | 69,961 |

Peräkkäiset keskiöetäisyydet noin 27,200 / 28,480 / 31,040 mm. Akselit eivät ole suorassa rivissä. Ratastasot Z: A 5,6–12,0 mm; B 16,0–22,4 mm; C 26,4–32,8 mm.

## Elektroniikan tilavaraukset

- PP3 / 9 V paristo: 48,5 × 26,5 × 17,5 mm.
- Arduino Nano: 45 × 18 × 7 mm, pariston yläpuolella.
- Mikrokytkin: 20 × 10 × 6 mm.
- Pieni näyttö: 25 × 12 × 4 mm.

Tilavaraukset eivät vahvista todellisten johtojen, liittimien, paristonepparin tai kiinnittimien mahtumista. Laskurin sähköistä toteutusta ei ole mukana.

## Valmius ja tarkistukset

Alkuperäisessä v4-tarkistusraportissa on kolmelle ratasparille 2D-hammasprofiilien törmäystarkistus yhden hammasjakson yli sekä osien 03 ja 04 STL-eheystarkistukset. Ne eivät ole fyysinen kuormitus- tai käyttökoe. Vientivaiheen tavutarkan lähdevastaavuuden ja STEP-tiedostojen solidi-entiteettimäärien tarkistukset ovat vientitarkistus.json-tiedostossa.

Akselit, rattaiden lukitus akseleihin, aksiaalinen pidätys, kiinnittimet, mahdolliset laakerit ja kumimoottorin koukku eivät sisälly pääkokoonpanon geometriaan. Nimellinen Ø5 mm reikä ei vielä määrittele sovitetta tai momenttia välittävää liitosta. Nämä on ratkaistava ennen toimivan laitteen kokoamista. STEP-vientiä ei tule tulkita valmiiksi valmistus- ja kuormitustestatuksi tuotteeksi.
