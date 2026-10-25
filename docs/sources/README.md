# Source Materials — UR Hardware Research

Original documents collected during the UR hardware research (2026-10-25) that backs
[`../ur_hardware_research.md`](../ur_hardware_research.md). All files were fetched from
public sources by the research pass; `.txt` files are plain-text extractions of the
accompanying PDF for grep-ability.

**Total size: ~100 MB.** The large files (manuals, theses) bloat the git history if
committed — consider `.gitignore` + git-LFS or keeping this directory untracked.

## vendor/

| File | What it is | Source |
|---|---|---|
| `UR5e_User_Manual.pdf` | UR5e User Manual v5.8 (243 pp.) — joint limits, safety functions, tool I/O, stop behavior | <https://s3-eu-west-1.amazonaws.com/ur-support-site/69091/99404_UR5e_User_Manual_en_Global.pdf> |
| `UR5_CB3_User_Manual.pdf` | UR5/CB3 User Manual 3.4.5 — CB3 error codes, safety limits, monitored-stop numbers | <https://s3-eu-west-1.amazonaws.com/ur-support-site/105370/99202_UR5_User_Manual_en_Global.pdf> |
| `UR_eSeries_Service_Manual.pdf` | e-Series Service Manual v1.1.6 — full error-code tables, brakes, selftest | <http://www.dour-robots.com/upload/down/UR%E6%9C%BA%E5%99%A8%E4%BA%BA%E6%9C%8D%E5%8A%A1%E6%89%8B%E5%86%8C/ServiceManual_EN_e-Series_V_1.1.6.pdf> |
| `UR3e_Datasheet.pdf` / `UR5e_Datasheet.pdf` / `UR10e_Datasheet.pdf` / `UR16e_Datasheet.pdf` | Current e-series datasheets (incl. FTS ranges/precision/accuracy, speeds, payload) | <https://www.universal-robots.com/manuals/latest/en/datasheets/ur3e/> (and ur5e/ur10e/ur16e) |
| `UR5e_Factsheet_DE.pdf` | German UR5e product factsheet (cross-check of FTS figures) | <https://www.universal-robots.com/media/1811480/ur5e-product-factsheet-de-web.pdf> |
| `Renishaw_AksIM_UR_CaseStudy.pdf` | Renishaw case study: AksIM absolute magnetic encoders in UR robots | <https://www.renishaw.com/nl/aksim-supports-universal-robots-for-smart-factory-automation--40389> |
| `RTDE_Guide.html` | Official RTDE guide (500 Hz data, `joint_temperatures`, `actual_tool_accelerometer`) | <https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/> |
| `UR_MaxOperatingTemperature_Article.html` | UR support article on operating temperature / logging thresholds | <https://www.universal-robots.com/articles/ur/application-installation/what-is-the-maximum-temperature-the-robot-can-work-in/> |

## error_codes/

UR error-code handbook pages (PolyScope 5.26 = SW5_26, newer generation = SW10_15),
base URL <https://www.universal-robots.com/manuals/EN/HTML/SW5_26/Content/prod-err-codes/topics/>.

| File | Content |
|---|---|
| `C62_ThermalIssue_SW5_26.html` / `C62_ThermalIssue_SW10_15.html` | Joint temperature high (80 °C) / shut down (85 °C) |
| `C75_JointEncoderError.html` | Joint encoder error sub-flags (invalid decode, signal, temperature…) |
| `C261_Temperature.html` | Temperature errors incl. power-sensor/SCB overheat |
| `C265_JointEncoder.html` | e-Series joint encoder error sub-flags |
| `C310_JointTemperatureManager.html` | Joint temperature manager errors |
| `C353_IMU_SW5_26.html` / `C353_IMU_SW10_15.html` | Base Filter Board IMUs (ISM330IS, ICM42688) |
| `C363_ToolAccelerometer_SW5_26.html` / `C363_ToolAccelerometer_SW10_15.html` | Tool PCBA accelerometer |
| `C450_ForceTorqueSensor.html` | FTS error family |
| `C453_SCB_IMU_SW10_15.html` | Safety Control Board IMU (newer generation) |

## literature/

| File | What it is | Source |
|---|---|---|
| `Madsen_PhD_Thesis_Aarhus.pdf` | Madsen PhD thesis (Aarhus Univ.) — e-series joint architecture (dual encoders, current/temp sensors), strain-wave stiffness/backlash | <https://pure.au.dk/ws/files/197603831/Phd_Thesis_Emil_Madsen.pdf> |
| `PoliTo_UR5_Joint_Encoders_Thesis.pdf` | Politecnico di Torino thesis — UR5 joint teardown, motor encoder (1080 CPR), gearbox | <https://webthesis.biblio.polito.it/16939/1/tesi.pdf> |

## Not preserved (failed or non-document downloads)

- Gnad et al. IEEE Access 2026 full text (paywalled — only abstract captured; DOI
  <https://doi.org/10.1109/ACCESS.2026.3726682>)
- UoP FTS-modeling thesis (server 503; repository URL in the research doc)
- Robotica 2024 "Electromechanical modeling of the UR5 e-Series" (paywalled)
- Guida et al., *Machines* 2024 (open access, not downloaded — see
  <https://www.mdpi.com/2075-1702/12/11/776>)
- PoliTo gearbox-fault thesis 22398 (secure link, not downloaded)
- Search-engine result dumps (`search.json`, `tree*.json`, `doaj.json`, etc.) — discarded
