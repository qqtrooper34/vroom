# TAMS Custom Fork of VROOM

История кастомных модификаций нашего форка VROOM поверх upstream master.

## Текущее состояние

- **Production**: ветка `my-customizations`, бинарь `/home/trooper34/vroom/bin/vroom`. Порт 3000 (PM2 process `TAMSrouting`).
- **Test/Staging**: ветка `upstream-rebase` (наши коммиты на upstream master, включая `service_per_type`), бинарь `/home/trooper34/vroom-bin-merged/vroom`. Порт 3001 (`TAMSrouting-test`).

После пакетного теста на стейдже test → катим в prod (заменяем `/home/trooper34/vroom/bin/vroom`, restart PM2).

## Кастомные коммиты (`my-customizations` ↔ `upstream-rebase`)

| Hash (my-customizations) | Hash (rebase) | Что | Файлы |
|---|---|---|---|
| `0974a0d7` | `84ebb0d0` | **fast_delivery / return_factor** — 0–100 множитель цены возврата на склад | vehicle.cpp/h, input_parser, solution_state |
| `575826b7` | `1460daba` | **fix пересчёт** — фолбэк для all-LAST jobs, route_position checks для shipments | heuristics, insertion_search, top_insertions, or_opt, relocate |
| `c472a0d8` | `2af92186` | фикс time_window — человекочитаемое сообщение об ошибке (local time + unix) | time_window.cpp |
| `43a24426` | `(сохранён)` | **max_work_time** — общий лимит рабочего времени (travel + service) | typedefs, eval.h, vehicle.cpp/h, input.cpp, helpers.cpp/h, ~7 CVRP операторов, choose_ETA, output_json |
| `385fcbc2` | `(сохранён)` | эвристика — приоритет FIRST jobs в init | heuristics.cpp (fill_route block) |
| `2a709865` | `(сохранён)` | **route_position** ("first"/"last") — позиционные ограничения jobs | ~25 файлов: input_parser, job, heuristics, insertion_search, top_insertions, choose_ETA, swap_star, ~12 CVRP операторов |
| `3fa175b6` | (выпущен при rebase) | Russian translations error messages | input_parser, helpers, input, choose_ETA, http_wrapper и пр. |

При rebase upstream master автоматически дропнул 6 наших коммитов как уже-в-upstream:
`5dcb23ee, cc95eec5, a75e33a6, 5e340d4a, 1622a941, 3b8ef440`.

Коммит `3fa175b6` (русские переводы) был **пропущен** при rebase, затем **re-applied 2026-06-07** (см. ниже раздел «Русские переводы»). Содержит только error-strings (UX), не функционально критично.

## Кастомные API-поля (наши + upstream)

### Vehicle
| Поле | Тип | Происхождение | Описание |
|---|---|---|---|
| `max_work_time` | UserDuration (опц.) | TAMS | Лимит общего рабочего времени (travel + service) |
| `return_factor` | unsigned 0-100 | TAMS | Множитель стоимости возврата на склад (default=100). Меньше — дешевле «быстрая доставка» |
| `type` | string | upstream | Идентификатор типа машины для type-aware service times |

### Job / Pickup / Delivery
| Поле | Тип | Происхождение | Описание |
|---|---|---|---|
| `route_position` | string "first"/"last" | TAMS | Позиционное ограничение (только в начале / только в конце маршрута) |
| `service_per_type` | obj `{type: service_seconds}` | upstream | Service duration override по типу машины |
| `setup_per_type` | obj `{type: setup_seconds}` | upstream | Аналогично для setup |

### Eval (internal)
| Поле | Происхождение | Описание |
|---|---|---|
| `service` | TAMS | Накопленный service time для проверки max_work_time |

## Reference golden set

`/tmp/vroom-ref/req_*.json` — 5 payload'ов из `tams_planning_logs` (test DB), `golden_*.json` — ответы текущего production-бинаря. Использовать для regression-теста при следующем upstream-merge:

```bash
for pk in 10 28 33 38 39; do
  curl -s -X POST http://localhost:3001/ -H 'Content-Type: application/json' \
    --data-binary @/tmp/vroom-ref/req_${pk}.json -o /tmp/merged_${pk}.json
done
```

Сравнение `summary.cost`/`duration`/`service`/`routes.length`/`unassigned.length`. Отклонения в COST допустимы при upstream-улучшениях heuristics (часто получается лучший cost — это OK).

## Synthetic tests для service_per_type

Test 1 (PASS at upstream-rebase, **FAIL at my-customizations**):
```json
{
  "vehicles": [{"id":1, "type":"brigade", "start":[37.60,55.75], "end":[37.60,55.75]}],
  "jobs": [{"id":100, "location":[37.61,55.76], "service":600, "service_per_type":{"brigade":120}}]
}
```
Ожидаемо: `routes[0].steps[job].service == 120`.

## Процесс следующего upstream-merge

1. **Зафиксировать reference golden** (см. выше) на текущем prod-бинаре.
2. `git checkout -b upstream-merge-NNN my-customizations && git rebase master`
3. Резолвить конфликты по одному коммиту. Используем стратегии:
   - **«take master»** для конфликтов в crash-heavy местах где upstream рефакторил полностью.
   - **«add both»** для дополняющих изменений (наш route_position + upstream service_per_type).
   - **«port hunks»** — взять master версию + ручная подача наших патчей.
4. После каждой группы — `make -j4` в `src/`, прогон reference golden.
5. Готовый бинарь сначала в `/home/trooper34/vroom-bin-merged/`, прогон тестов через `TAMSrouting-test` (порт 3001).
6. После проверки — заменить `/home/trooper34/vroom/bin/vroom`, restart `TAMSrouting` (порт 3000).
7. ВНИМАНИЕ: `make` в `/home/trooper34/vroom/src` пишет бинарь в `../bin/`. Если делаешь экспериментальную сборку — обязательно делай это на отдельной ветке и потом сохраняй бинарь в безопасное место **до того как пересобирать prod-ветку**.

## Полезные заметки про refactor 2024-2025

Master VROOM (после `5dee4816`) рефакторил:
- `Job.service` (single) → `Job.services[type_idx]` (вектор по типу машины). Нужно везде использовать `services[vehicle.type]`.
- Inline seeding в `basic()`/`dynamic_vehicle_choice()` → shared `seed_route()` + `fill_route()`.
- `Eval::operator==` → `= default` (C++20).
- Vehicle threading → unified `solving_threads`.

Если наш коммит модифицирует ЛЮБОЕ из этого — придётся переписывать под новый интерфейс.

---
Last updated: 2026-06-06 (upstream-rebase ветка содержит master HEAD `cd91c90b` + наши 7 коммитов).

## Acceptance test суммарно (20 сценариев на merged binary)

Прогон от 2026-06-06 на TAMSrouting-test (порт 3001):

**18 PASS / 3 FAIL / 21 TOTAL** (тест 9 разбит на 9a+9b).

| # | Тест | Статус |
|---|---|---|
| 1 | baseline 5 jobs solo | ✅ PASS |
| 2 | brigade type + service_per_type=75 | ✅ PASS |
| 3 | mixed brigade(cheaper) vs solo — brigade берёт все 3 jobs за 150s | ✅ PASS |
| 4 | route_position FIRST | ✅ PASS |
| 5 | route_position LAST | ✅ PASS |
| 6 | mixed FIRST/NONE/LAST | ✅ PASS |
| 7 | all-LAST fallback (3 LAST jobs все размещены) | ✅ PASS |
| 8 | max_work_time=600 enforce | ❌ FAIL — НЕ enforce'ится в путях вне fill_route |
| 9a | return_factor=100 baseline | ✅ PASS |
| 9b | return_factor=50 уменьшает cost | ✅ PASS (3460 < 4517) |
| 10 | shipment pickup→delivery порядок | ✅ PASS |
| 11 | shipment delivery route_position=last | ❌ FAIL — позиция не enforce'ится для PD |
| 12 | multi-brigade-types (brigade-a быстрее brigade-b) | ✅ PASS |
| 13 | type "unknown" → fallback на service | ✅ PASS |
| 14 | no vehicle.type → service_per_type игнорируется | ✅ PASS |
| 15 | capacity ограничение (1000 vs 3×600) | ✅ PASS |
| 16 | skills routing | ✅ PASS |
| 17 | brigade+skills | ⚠️ Не баг — VROOM консолидирует jobs в одну машину когда одна обязательна по skill |
| 18 | 10 jobs brigade vs solo (одинак. costs) | ⚠️ Не баг — VROOM выбрал tied-cost; с явно дешёвой бригадой PASS |
| 19 | TW + brigade service_per_type | ✅ PASS |
| 20 | combined brigade+FIRST+LAST+work_time+return | ✅ PASS |

### Реальные баги для follow-up

**Баг 1: max_work_time не enforce'ится во всех путях** (test 8)

Проявление: vehicle с `max_work_time=600` принял маршрут с `duration=2921, service=1200`. Должен был оставить часть jobs в unassigned.

Причина: наш `ok_for_range_bounds` проверяет `(e.duration + e.service) ≤ max_work_time`, но upstream's refactored CVRP operators не накапливают `e.service` через `addition_cost`. В master'е `Eval` не имеет нашего `service` поля по дизайну, и `helpers.h::addition_cost` его выставляет ТОЛЬКО для single-job insertion, не для PD shipments / route-level evaluations / в operators.

Что нужно: добить `e.service` во всех путях evaluation Job (route_eval_for_vehicle, и в каждом операторе при подсчёте gain). Это ~30+ файлов.

**Баг 2: shipment delivery route_position=last не enforce'ится** (test 11)

Проявление: shipment с `delivery.route_position="last"` поставлен в середину (steps: pickup, job1, **delivery**, job2). job2 после delivery.

Причина: `is_valid_rank_for_route_position` помещён в `compute_best_insertion_single` (single jobs) и в `compute_best_insertion_pd` (shipment pickup и delivery). Но в эвристике `fill_route` для shipment ветки `JOB_TYPE::PICKUP` нет вызова `is_valid_route_position`.

Что нужно: добавить проверку route_position в shipment-insertion ветке `fill_route` (для pickup_r и delivery_r отдельно).

### Тестов прошедших с малой модификацией (не баги)

- **17b**: brigade с `per_hour=1800` vs solo `per_hour=7200` — j1 всё равно ушёл на solo, потому что j2 (skill=20) требует solo обязательно, и VROOM экономит один fixed_cost ($500), сажая обе jobs на solo. Это правильное cost-based решение.
- **18b**: brigade с явно меньшим cost'ом → берёт все 10/10 jobs (cost 4274). Подтверждает: при clear cost advantage VROOM предпочитает brigade.

## Русские переводы (re-applied 2026-06-07)

Исходный коммит `3fa175b6` (русские translations error messages) был **пропущен** при первичном rebase и переприменён позже отдельной партией.

**Что переведено** (всего 60+ строк, 0 английских осталось):
- `Exception("...")` строковые литералы (heuristics, choose_ETA, http_wrapper, libosrm/osrm_wrappers, cost_wrapper, input, time_window, input_parser, helpers, main.cpp)
- `Exception(std::format("...{}..."))` строки (vehicle.cpp:71, input.cpp:193/234/237, choose_ETA.cpp:995/1044)
- Все новые строки, появившиеся в upstream после исходного 3fa175b6 (VROOM compiled without ..., No vehicle/task defined, Empty matrices, Unexpected matrix line length, Input root is not an object, и т.д.)

**Где словарь** (для будущих re-rebase):
- `i18n/ru.tsv` — TSV с парами `<English>\t<Russian>`, отсортирован по English. При следующем upstream-merge: прогнать словарь по тому же набору файлов через node-скрипт (см. ниже).

**Применение словаря** при следующем rebase:
```bash
cd /home/trooper34/vroom
node -e "
const fs = require('fs');
const pairs = fs.readFileSync('i18n/ru.tsv', 'utf8').trim().split('\n').map(l => l.split('\t'));
const files = ['src/algorithms/heuristics/heuristics.cpp', 'src/algorithms/validation/choose_ETA.cpp',
  'src/routing/http_wrapper.cpp', 'src/routing/libosrm_wrapper.cpp', 'src/routing/osrm_routed_wrapper.cpp',
  'src/structures/vroom/cost_wrapper.cpp', 'src/structures/vroom/input/input.cpp', 'src/structures/vroom/time_window.cpp',
  'src/structures/vroom/vehicle.cpp', 'src/utils/helpers.cpp', 'src/utils/input_parser.cpp', 'src/main.cpp'];
for (const f of files) {
  let s = fs.readFileSync(f, 'utf8'); let c = 0;
  for (const [en, ru] of pairs) {
    const n = '\"' + en, r = '\"' + ru;
    if (s.includes(n)) { c += s.split(n).length - 1; s = s.split(n).join(r); }
  }
  if (c > 0) { fs.writeFileSync(f, s); console.log(f, c); }
}
"
```
После прогона: `grep -rEn '(Input|Routing)Exception\(' src/ --include='*.cpp' --include='*.h' | grep '\"[A-Za-z]' | grep -v 'VROOM\|libOSRM'` должен дать пусто.

**Проверка перевода работает**:
```bash
echo '{"vehicles":[{"id":1}],"jobs":[{"id":2,"location":[37.6,55.7]}]}' | bin/vroom
# → {"code":2,"error":"Не указан старт или конец для транспорта 1."}
```
