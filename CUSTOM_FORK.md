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

Коммит `3fa175b6` (русские переводы) был **пропущен** при rebase — нужно отдельно re-applied после стабилизации. Содержит только error-strings (UX), не функционально критично.

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
