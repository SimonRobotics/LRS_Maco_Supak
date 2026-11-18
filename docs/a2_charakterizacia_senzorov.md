# A2 §1 — Charakterizácia senzorov

Zadanie: [localization.md §1](../assignments/assignment2/localization.md#1-sensor-acquisition-and-characterisation--15-points).
Odhadovač ([`lrs_localization`](../lrs_localization/README.md)) odoberá iba `/localization/gps`,
`/localization/vo` a `/localization/imu`. Ground truth `/evaluation/ground_truth` číta len offline
skript [`characterise.py`](../lrs_localization/scripts/characterise.py), ktorý kreslí grafy z bagov
a nič neposiela späť do odhadovača ani regulátora.

## Nahrávky

| Bag | Seed (Gazebo aj senzory) | Čo dron robí | Dĺžka |
|---|---|---|---|
| [`bags/a2_static_seed42`](../bags/a2_static_seed42) | 42 | stojí na zemi | 59 s |
| [`bags/a2_static_seed42_repeat`](../bags/a2_static_seed42_repeat) | 42 | stojí, nové spustenie celej simulácie | 59 s |
| [`bags/a2_static_seed73`](../bags/a2_static_seed73) | 73 | stojí | 59 s |
| [`bags/a2_flight_seed42`](../bags/a2_flight_seed42) | 42 | 2 kolá obdĺžnika z §4 vo výške 2 m, 0,5 m/s | 111 s |

Prelet riadil náš A1 uzol `lrs_mission` s misiou [`missions/a2_rectangle.csv`](../missions/a2_rectangle.csv)
na polohe z MAVROS — iba aby sa dron hýbal počas nahrávania. Odhadovač z A2 pri tom nebol zapojený.

Nahrávanie (rekordér sa spúšťa pred senzorovým simulátorom, aby zachytil aj prvú vzorku):

```bash
gz sim -r -v 4 --seed 42 "$PWD/worlds/fei_lrs_gazebo_localization.world"
ros2 bag record -s mcap --storage-preset-profile zstd_fast -o bags/a2_static_seed42 \
  /localization/gps /localization/vo /localization/imu /evaluation/ground_truth
ros2 launch localization_sensor_sim localization_sensors.launch.py seed:=42
# + SITL a MAVROS; pri prelete aj:
ros2 run lrs_mission lrs_mission_node --ros-args --params-file install/lrs_mission/share/lrs_mission/config/indoor.yaml \
  -p mission_file:=$PWD/missions/a2_rectangle.csv \
  -p map_file:=$PWD/install/lrs_mission/share/lrs_mission/maps/FEI_LRS_PCD/map.pcd -p max_speed:=0.5
```

Grafy a čísla:

```bash
python3 lrs_localization/scripts/characterise.py bags/a2_static_seed42 bags/a2_flight_seed42 --out docs/figures/a2
python3 lrs_localization/scripts/characterise.py --compare \
  bags/a2_static_seed42 bags/a2_static_seed42_repeat bags/a2_static_seed73 --out docs/figures/a2
```

## Frekvencie, framy a časové pečiatky

Frekvencia je počítaná z `header.stamp` (simulačný čas), nie z času príchodu. Gazebo bežal na
~0,9× reálneho času, takže `ros2 topic hz` (wall time) ukazuje menej — z pečiatok je to presne:

| Topic | Frekvencia | Medián Δt | Frame → child | Nemonotónne pečiatky |
|---|---:|---:|---|---:|
| `/localization/gps` | 5,00 Hz | 0,200 s | `map` | 0 |
| `/localization/vo` | 30,00 Hz | 0,030 s | `vo_odom` → `base_link` | 0 |
| `/localization/imu` | 99,9 Hz | 0,010 s | `base_link` | 0 |

Rovnaké hodnoty vypisuje aj samotný uzol odhadovača za behu (riadok `[sensors] ...`).

## GPS — šum

![GPS](figures/a2/a2_flight_seed42_gps.png)

Chyba GPS − skutočná poloha (prelet, 549 vzoriek):

| | x | y | z |
|---|---:|---:|---:|
| priemer [m] | 0,00 | 0,02 | −0,13 |
| smerodajná odchýlka [m] | 0,62 | 0,61 | 0,92 |
| konfigurácia simulátora [m] | 0,6 | 0,6 | 0,9 |

Šum je bez viditeľného posunu a nezávislý od vzorky k vzorke → GPS je absolútna, ale hlučná
poloha. Kovariancia v správe (`pose.covariance` diagonála 0,36 / 0,36 / 0,81 m²) zodpovedá
nameranému rozptylu; orientácia má kovarianciu 10⁶, t. j. GPS yaw nemeria.

## VO — drift

![VO statický](figures/a2/a2_static_seed42_vo.png)
![VO prelet](figures/a2/a2_flight_seed42_vo.png)

| Nahrávka | Vodorovný drift | Drift z | Drift yaw |
|---|---:|---:|---:|
| stojí, 59 s | 0,80 m | +0,45 m | −2,9° |
| prelet, 110 s | 0,46 m | +0,69 m | −5,9° |

- Poloha VO je hladká, ale **uteká aj keď dron stojí** (bias rýchlosti) — po minúte je chyba väčšia
  ako šum GPS. Polohu z VO preto nemožno brať ako absolútnu polohu v `map`.
- Kovariancia v správe VO to nezachytí: `pose.covariance` udáva σ = 1 cm (len výstupný šum, nie
  drift), `twist.covariance` σ = 0,03 m/s — to druhé so skutočnosťou sedí (graf vpravo).
- Pri prelete drift vodorovne čiastočne kompenzuje (bias je v body frame a otáča sa s dronom),
  yaw a z ale rastú monotónne.
- **Lineárna rýchlosť VO je v `base_link` (body frame).** Počas pohybu sa líši od skutočnej rýchlosti
  otočenej do body frame o RMS **0,06 m/s**, kým od skutočnej rýchlosti v ENU o **0,66 m/s** — pravý
  graf: pri lete na juh (yaw −90°) je ENU `vx` nulová, ale body `vx` = +0,5 m/s.

## Gyroskop — bias

![Gyro](figures/a2/a2_static_seed42_gyro.png)

| Nahrávka | Bias z (priemer ω_z − skutočná rýchlosť) | Šum (σ) |
|---|---:|---:|
| seed 42, stojí | −0,0170 rad/s | 0,0040 rad/s |
| seed 42, prelet | −0,0170 rad/s | 0,0051 rad/s* |
| seed 73, stojí | −0,0239 rad/s | — |

\* pri prelete je v σ aj chyba numerickej derivácie skutočného yaw v zákrutách.

Keď sa dron neotáča, `angular_velocity.z` by mala byť 0 — priemer −0,017 rad/s je statický bias
(≈ −1°/s, za minútu by integrovaný yaw ušiel o ~58°). Rozptyl v správe
(`angular_velocity_covariance[8]` = 1,6·10⁻⁵, σ = 0,004 rad/s) sedí s meraním. Orientácia a zrýchlenie
majú kovarianciu −1, t. j. nie sú k dispozícii.

## Seed — opakovateľnosť

![Seeds](figures/a2/seeds.png)

| Porovnanie | GPS šum (priemer \|rozdielu\| sekvencií) | Gyro |
|---|---:|---|
| seed 42 vs. seed 42 (nové spustenie) | 0,0000 m | max \|Δω_z\| 1,2·10⁻¹⁴ rad/s pri 5784 rovnakých pečiatkach |
| seed 42 vs. seed 73 | 0,62 m | bias −0,0170 vs. −0,0239 rad/s |

Rovnaký seed zopakuje presne tú istú koróziu (GPS aj IMU), iný seed zmení šum aj skrytý bias.
Filter preto nesmie bias predpokladať — musí ho odhadovať.

## Čo z toho plynie pre filter (§2)

- **GPS**: meranie polohy `z = [x, y, z]`, `R_gps = diag(0,6², 0,6², 0,9²)` m² (z kovariancie v správe).
- **Gyro**: vstup do predikcie yaw, `ψ̇ = ω_z − b`; šum σ = 0,004 rad/s ide do `Q` pre yaw, bias `b`
  je stav filtra (náhodná prechádzka s malým `Q`).
- **VO**: kvôli driftu nepoužívať absolútnu polohu VO; použiť **rýchlosť v body frame**
  (s rotáciou o odhadovaný yaw), prípadne prírastky polohy/yaw. Rýchlosť z VO viaže `vx, vy, vz`,
  ktoré GPS (5 Hz, σ 0,6 m) určuje len slabo.
- **Pečiatky**: všetky tri senzory majú monotónne pečiatky v spoločnom simulačnom čase, takže
  predikciu možno robiť na `Δt` z pečiatok a merania aplikovať asynchrónne.
