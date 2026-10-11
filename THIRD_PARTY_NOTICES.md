# Source and third-party notices

Project source is GPL-3.0-only unless identified below. ESPHome and toolchain
components are fetched during the build and retain their own licences; no vendor
firmware, bootloader dump or SDK binary is distributed in this repository.

## Radio

The receive-only serial transport in `cmt2300a.h` was rewritten using CMOSTEK's
CMT2300A datasheet (English, revision 0.7, serial-interface specification)
and the MIT [anisyanka/cmt2300a driver](https://github.com/anisyanka/cmt2300a/tree/eb7be847a1f415ff40230ef1a7712897c139e1ec)
by Aleksandr Anisimov. Its notice is in `LICENSES/CMT2300A-MIT.txt`.

`radio_profile.h` contains the numeric register settings needed to interoperate
with the radio hardware and Fine Offset transmissions. These settings originated
in analysis of the devices' factory configuration, including undocumented
calibration values; they are retained as hardware interface data. They are not a
firmware payload. 

## Fine Offset decoders

`fineoffset.h` contains C++ ports of the following rtl_433 sources, modified
for ESPHome payload parsing, validation and aggregation on 2026-10-10.
Their GPL-2.0-or-later terms permit use under GPLv3; original notices follow.
The GPLv2 text is retained in `LICENSES/GPL-2.0.txt`.

## ambientweather_wh31e.c

Source: https://github.com/merbanan/rtl_433/blob/d511d8a2f8ea5bd790491cc31fa2a799ff46b7d8/src/devices/ambientweather_wh31e.c

```c
/** @file
    Ambient Weather WH31E, EcoWitt WH40, EcoWitt WN20 protocol.

    Copyright (C) 2018 Christian W. Zuckschwerdt <zany@triq.net>
    based on protocol analysis by James Cuff and Michele Clamp,
    EcoWitt WH40 analysis by Helmut Bachmann,
    Ecowitt WS68 analysis by Tolip Wen improved by Bruno Octau,
    EcoWitt WH31B analysis by Michael Turk,
    EcoWitt WN20 analysis by Ryan Romanchuk.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
```


## fineoffset.c

Source: https://github.com/merbanan/rtl_433/blob/d511d8a2f8ea5bd790491cc31fa2a799ff46b7d8/src/devices/fineoffset.c

```c
/** @file
    Fine Offset Electronics sensor protocol.

    Copyright (C) 2017 Tommy Vestermark
    Enhanced (C) 2019 Christian W. Zuckschwerdt <zany@triq.net>
    Added WH51 Soil Moisture Sensor (C) 2019 Marco Di Leo

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
```


## fineoffset_wh31l.c

Source: https://github.com/merbanan/rtl_433/blob/d511d8a2f8ea5bd790491cc31fa2a799ff46b7d8/src/devices/fineoffset_wh31l.c

```c
/** @file
    Ambient Weather (Fine Offset) WH31L protocol.

    Copyright (C) 2021 Christian W. Zuckschwerdt <zany@triq.net>
    based on protocol analysis by \@MksRasp.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
```


## fineoffset_wh43.c

Source: https://github.com/merbanan/rtl_433/blob/d511d8a2f8ea5bd790491cc31fa2a799ff46b7d8/src/devices/fineoffset_wh43.c

```c
/** @file
    Fine Offset Electronics WH43 air quality sensor.

    Analysis by \@andrewjmcginnis
    Copyright (C) 2025 Christian W. Zuckschwerdt <zany@triq.net>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
```


## fineoffset_wh45.c

Source: https://github.com/merbanan/rtl_433/blob/d511d8a2f8ea5bd790491cc31fa2a799ff46b7d8/src/devices/fineoffset_wh45.c

```c
/** @file
    Fine Offset Electronics WH45 air quality sensor.

    Copyright (C) 2022 \@anthyz

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
```


## fineoffset_wh46.c

Source: https://github.com/merbanan/rtl_433/blob/d511d8a2f8ea5bd790491cc31fa2a799ff46b7d8/src/devices/fineoffset_wh46.c

```c
/** @file
    Fine Offset Electronics WH46 air quality sensor.

    Based on fineoffset_wh45 from \@anthyz
    Copyright (C) 2024 \@joanma747

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
```


## fineoffset_wh52.c

Source: https://github.com/merbanan/rtl_433/blob/d511d8a2f8ea5bd790491cc31fa2a799ff46b7d8/src/devices/fineoffset_wh52.c

```c
/** @file
    Fine Offset Electronics / Ecowitt WH52 3-in-1 Soil Moisture / Temperature / EC sensor.

    Copyright (C) 2026 Andreas Braunlich <andreasbraunlich@gmail.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
```


## fineoffset_wh55.c

Source: https://github.com/merbanan/rtl_433/blob/d511d8a2f8ea5bd790491cc31fa2a799ff46b7d8/src/devices/fineoffset_wh55.c

```c
/** @file
    Fine Offset / Ecowitt WH55 water leak sensor.

    Copyright (C) 2023 Christian W. Zuckschwerdt <zany@triq.net>
    Protocol analysis by \@cdavis289, test data by \@AhrBee

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
 */
```


## fineoffset_wn34.c

Source: https://github.com/merbanan/rtl_433/blob/d511d8a2f8ea5bd790491cc31fa2a799ff46b7d8/src/devices/fineoffset_wn34.c

```c
/** @file
    Fine Offset Electronics WN34 Temperature Sensor.

    Copyright (C) 2022 \@anthyz

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
```


## fineoffset_ws80.c

Source: https://github.com/merbanan/rtl_433/blob/d511d8a2f8ea5bd790491cc31fa2a799ff46b7d8/src/devices/fineoffset_ws80.c

```c
/** @file
    Fine Offset Electronics WS80 weather station.

    Copyright (C) 2022 Christian W. Zuckschwerdt <zany@triq.net>
    Protocol description by \@davidefa

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
```


## fineoffset_ws85.c

Source: https://github.com/merbanan/rtl_433/blob/d511d8a2f8ea5bd790491cc31fa2a799ff46b7d8/src/devices/fineoffset_ws85.c

```c
/** @file
    Fine Offset Electronics WS85 weather station.

    Copyright (C) 2022 Christian W. Zuckschwerdt <zany@triq.net>
    Protocol description by \@davidefa

    Copy of fineoffset_ws90.c with changes made to support Fine Offset WS85
    sensor array.  Changes made by John Pochmara <john@zoiedog.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
```


## fineoffset_ws90.c

Source: https://github.com/merbanan/rtl_433/blob/d511d8a2f8ea5bd790491cc31fa2a799ff46b7d8/src/devices/fineoffset_ws90.c

```c
/** @file
    Fine Offset Electronics WS90 weather station.

    Copyright (C) 2022 Christian W. Zuckschwerdt <zany@triq.net>
    Protocol description by \@davidefa

    Copy of fineoffset_ws80.c with changes made to support Fine Offset WS90
    sensor array.  Changes made by John Pochmara <john@zoiedog.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
```
