# License scope and third-party material

The root [LICENSE](LICENSE) contains the MIT license for project-owned software
and documentation, copyright (c) 2026 Ali Hubail, Abdulla Yusuf, Mujtaba Obaid.
Keep its copyright and permission notices with redistributed copies or
substantial portions. The text follows the [OSI MIT license](https://opensource.org/license/mit).

Third-party libraries, publications, institutional documents, CAD models, and
visual assets retain their own rights and terms. Their presence in this
repository does not place them under the project's MIT license. A citation or a
source link does not establish redistribution permission.

## Python application and Windows releases

The Python application uses CustomTkinter, NumPy, SciPy, pandas, Matplotlib,
Pillow, openpyxl, and pyserial, together with their dependencies and the
Python/Tcl/Tk runtime. Consult the license files supplied with the exact versions
used in a release. NumPy and SciPy wheels, for example, can also contain notices
for bundled native libraries; Matplotlib contains font notices.

The manual build and GitHub build workflow run
`Python Apps/packaging/collect_licenses.py` after PyInstaller. It copies the
project license and this document next to `BHO.exe`, preserves installed package
license/notice files under `licenses/packages/`, and records exact installed
versions and license metadata in `licenses/inventory.json`. The collection covers
the build environment, including packages that may not be embedded in the app.
It also preserves the Python runtime license and locally available Tcl/Tk
notices. The installer installs these files and displays the project MIT license.

Before distributing an installer, review the inventory entries without separate
notice files and any asset-specific terms that are absent from package metadata.
The collector preserves available notices; it does not verify every embedded
binary or determine permissions. The separately bundled Microsoft Visual C++
runtime remains subject to Microsoft's redistributable terms. MATLAB, Simulink,
SolidWorks, and Multisim are separate tools with their own licenses; this
repository does not license or distribute those tools.

[PyInstaller's license exception](https://pyinstaller.org/en/stable/license.html)
allows generated bundles to use the application's chosen license while requiring
compliance with dependency licenses. It does not require changing this project's
MIT license to GPL. Python's terms are documented in the
[Python history and license](https://docs.python.org/3.12/license.html).

## Research publications and institutional documents

These files include third-party publications and material whose ownership or
permission needs confirmation:

| Repository material | Recorded rights and source | Status |
| --- | --- | --- |
| `Miscellaneous & Others/Useful Docs & Files/TF identification Part/robotics-12-00140.pdf` | Gonzalez-Villagomez et al., *An Experimental Study of the Empirical Identification Method to Infer an Unknown System Transfer Function*, Robotics 12(5), 140 (2023), [DOI: 10.3390/robotics12050140](https://doi.org/10.3390/robotics12050140). The PDF states copyright 2023 by the authors and [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). | Retain the authors, source, and CC BY notice when sharing; identify changes if made. |
| `.../algorithim part/Black_hole_A_new_heuristic_optimization.pdf` | A. Hatamlou, *Black hole: A new heuristic optimization approach for data clustering*, Information Sciences 222 (2013), 175–184, [DOI: 10.1016/j.ins.2012.08.023](https://doi.org/10.1016/j.ins.2012.08.023). The PDF states copyright 2012 Elsevier Inc., all rights reserved. | Public redistribution permission is not recorded in this repository. Verify permission or distribute a citation/link instead of the PDF. |
| `.../algorithim part/A_new_hybrid_algorithm_based_on_black_hole_optimization_and_bisecting_k-means_for_cluster_analysis.pdf` | IEEE 2014, [publisher record](https://ieeexplore.ieee.org/document/6999695). | The local PDF identifies an institutional download with restrictions. Public redistribution permission is not recorded. |
| `.../algorithim part/Economic_dispatch_incorporating_wind_energy_by_BH_BBO_and_DE.pdf` | IEEE 2016, [publisher record](https://ieeexplore.ieee.org/document/7726767). | The local PDF identifies an institutional download with restrictions. Public redistribution permission is not recorded. |
| `.../algorithim part/How_effective_is_Black_Hole_Algorithm.pdf` | IEEE 2016, [publisher record](https://ieeexplore.ieee.org/document/7918011). | The local PDF identifies an institutional download with restrictions. Public redistribution permission is not recorded. |
| `.../algorithim part/Black Hole.pdf` | Extract containing explanatory prose and a black-hole image; it cites Nola Taylor Tillman (2024). | Author, original visual source, and reuse permissions are not established by this extract. |
| `Miscellaneous & Others/Important Final Year Project Assessments/` | EN8914 assessment descriptions and `Thesis Format.docx`, associated with Bahrain Polytechnic. | No public reuse license or redistribution permission is recorded. These are not relicensed under MIT. |

The `.../algorithim part/` entries above refer to
`Miscellaneous & Others/Useful Docs & Files/algorithim part/`. The repository's
[References.txt](https://github.com/Abdulla-22/BHO-PID-Optimizer/blob/main/Miscellaneous%20%26%20Others/References.txt) and
[bibliography XML](https://github.com/Abdulla-22/BHO-PID-Optimizer/blob/main/Miscellaneous%20%26%20Others/uesd_sources.xml) supply source
links, but do not establish the rights to redistribute all downloaded material.

## CAD references

The references list these GrabCAD models. Their names correspond to repository
components, but exact file origin, creator attribution, and any individual
permission agreements still need to be confirmed:

| Recorded model reference | Repository area to verify |
| --- | --- |
| [Wooden Conveyor Belt](https://grabcad.com/library/wooden-conveyor-belt-1) | `Drawings & 3D Designs/Belt Conveyor/Wooden Conveyor/` |
| [Arduino Nano](https://grabcad.com/library/arduino-nano-7) | `Drawings & 3D Designs/Components/arduino nano.SLDPRT` |
| [Motor Driver BTS7960 43A](https://grabcad.com/library/motor-driver-bts7960-43a-1) | `Drawings & 3D Designs/Components/BTS7960.SLDPRT` |
| [KeyPad 3x4 with PCB](https://grabcad.com/library/keypad-3x4-w-pcb-1) | `Drawings & 3D Designs/Components/KeyPad_3x4_w_pcb.SLDPRT` |
| [LCD 1602](https://grabcad.com/library/lcd-1602-5) | `Drawings & 3D Designs/Components/LCD_1602.SLDPRT` |
| [Motor DC, Pololu](https://grabcad.com/library/motor-dc-pololu-1) | `Drawings & 3D Designs/Components/motorwithmount.SLDPRT` and `motorwithoutmount.SLDPRT` |
| [Switched-mode power supply](https://grabcad.com/library/switched-mode-power-supply-1) | `Drawings & 3D Designs/Components/power supply.SLDPRT` |

GrabCAD's [model-use guidance](https://help.grabcad.com/article/246-how-can-models-be-used-and-shared?locale=en)
requires original-creator attribution and a model link for non-commercial public
showcasing, and explicit original-designer permission for commercial public use.
The project's MIT license does not replace those terms. The bearing model named
`6001LU-NTNBearingCorp.ofAmerica-3D-03-25-2022` also needs its original source and
redistribution terms recorded; its filename alone does not establish permission.

## Existing visual assets

The origin and rights of the existing GIFs/PNGs in `Python Apps/images/` and the
matching MATLAB application images are not documented per asset. The same is
true for externally sourced visuals that may appear in
`Miscellaneous & Others/images/bb2.gif`, `Drawings & 3D Designs/prototype sample.png`,
and the research extracts. Verify their creators and source terms before
reusing them outside the project or including them in a new public release.
Photographs, screenshots, and videos should likewise be attributed to their
actual creators rather than assuming ownership from their filenames.

Original README diagrams generated from this repository's own explanation have
their editable generation source alongside the project documentation. Those
new project-owned assets follow the root MIT license.
