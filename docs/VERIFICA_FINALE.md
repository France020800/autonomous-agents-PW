# Verifica finale e consegna

## Esito

**Verifica automatica completata con successo**, con vincolo invariato **N ≥ 3**.

- Build della wheel `mas_formation-0.1.0-py3-none-any.whl` riuscita.
- Installazione non editable in un nuovo ambiente virtuale, senza pacchetti di sistema.
- Import verificato da `site-packages`, fuori dal checkout e senza `PYTHONPATH`.
- `pip check`: nessuna dipendenza incoerente.
- **44 test superati** sulla wheel installata, con test copiati in una directory temporanea indipendente dal progetto.
- **9 esecuzioni CLI** completate, con esportazione HTML.
- Comando di confronto reattivo–predittivo eseguito anche dalla wheel installata; metriche numeriche coerenti con quelle salvate entro una tolleranza assoluta di 1e-9.
- Hash dei sorgenti verificati, link locali e bilanciamento dei delimitatori Markdown/matematici controllati prima del commit.

L'ambiente temporaneo viene eliminato alla fine; `.venv/` del progetto non viene modificato.

## Ambiente effettivamente provato

Python **3.12.14**, macOS ARM64. Dipendenze principali: NumPy **2.5.5**, Matplotlib **3.11.2**, pytest **9.2.0**.

Sono conservati:

- [summary.json](results/validation/summary.json): esito, comandi con codici di uscita, versioni, confronto nell'ambiente pulito e hash SHA-256 di sorgenti, test e script verificati;
- [commands.log](results/validation/commands.log): log completo di build, installazione, test e avvii;
- [requirements.txt](results/validation/requirements.txt): versioni delle dipendenze dell'ambiente verificato.

Il supporto dichiarato resta Python 3.10+, ma questa verifica non è una matrice di certificazione per tutte le versioni Python o tutti i sistemi operativi. Eventuali variazioni nelle ultime cifre dei risultati con altre versioni delle librerie non indicano necessariamente una regressione.

## Scenari CLI verificati

Tutte le esecuzioni usano seed 42, passo 0.01 s, avoidance predittivo con orizzonte 1.5 s e 10 campioni futuri. Nei casi con ostacoli mobili ne sono presenti due; nel caso statico sono tre.

| Scenario | Agenti | Durata [s] |
|---|---:|---:|
| `formation` | 3 | 15 |
| `formation-goal` | 3 | 25 |
| `static-obstacles` | 3 | 25 |
| `moving-obstacles` | 3 | 30 |
| `dynamic-navigation` | 3 | 30 |
| `dynamic-navigation` | 4 | 30 |
| `moving-obstacles` | 5 | 30 |
| `formation-goal` | 6 | 25 |
| `formation` | 8 | 15 |

Questi sono smoke test di avvio/esecuzione/export, non prove che ogni scenario raggiunga sempre il goal. In particolare, quello statico può terminare lontano dall'obiettivo: è un limite noto degli APF, non un errore di installazione.

La suite verifica inoltre l'equivalenza al reattivo con orizzonte nullo, l'anticipo della repulsione in avvicinamento, i poligoni regolari e alcuni criteri numerici sui risultati.

## Come ripetere la verifica

Dalla radice del repository:

```bash
# Nuovo ambiente temporaneo, dipendenze risolte da pyproject.toml
python3 scripts/verify_clean_environment.py

# Per riusare le versioni già verificate (compatibilmente con Python e piattaforma)
python3 scripts/verify_clean_environment.py \
  --constraints docs/results/validation/requirements.txt
```

Servono accesso a PyPI oppure una cache pip completa. Ogni esecuzione aggiorna i tre file di evidenza; usare `--output-dir outputs/verifica_locale` per non sovrascrivere quelli di consegna.

La verifica usa il backend non interattivo Agg: controlla generazione delle figure ed esportazione dei dati HTML, ma **non attesta un collaudo visuale del browser o del backend grafico interattivo**.

## Confronto sperimentale conservato

Il [report del confronto](results/avoidance_comparison/report.md) comprende configurazione, metriche, curve, figure affiancate e traiettorie NPZ rianalizzabili.

Nel caso di navigazione a seed 42, N=3, durata 30 s:

- clearance minima reale campionata: **0.4134 m → 0.5597 m**;
- prima repulsione: **1.88 s → 1.04 s**;
- primo arrivo al goal: **17.95 s → 20.95 s**;
- nessun frame di contatto agente–ostacolo in entrambe le esecuzioni.

Il predittivo migliora la separazione in questo caso, ma allunga il tempo di arrivo. Un singolo seed non consente conclusioni statistiche generali. Nel caso senza goal la formazione predittiva ha ancora un piccolo errore residuo a 30 s, riportato nel JSON: il mantenimento della forma non è un vincolo esatto imposto dal simulatore.

## Limiti da dichiarare nella relazione

- Minimi locali ed equilibri indesiderati: nessuna garanzia di convergenza globale.
- Nessuna garanzia formale anticollisione, anche con predizione.
- Distanze predette rispetto alla posizione corrente dell'agente, senza previsione del suo moto.
- Campionamento finito delle previsioni; possibili discontinuità quando cambia il campione minimo.
- Saturazione, discretizzazione e clipping della clearance possono alterare le proprietà ideali di gradiente.
- Nessun avoidance dedicato tra agenti o tra ostacoli.
- Velocità degli ostacoli nota e costante; assenza di rumore e ritardi.
- Baricentro calcolato globalmente: navigazione non completamente distribuita.

## Pacchetto di consegna

L'archivio si può ricreare dal commit finale:

```bash
mkdir -p dist
git archive --format=zip --prefix=mas-formation/ \
  --output=dist/mas-formation-consegna.zip HEAD
```

Include sorgenti, test, script di verifica, documentazione e risultati riproducibili. Tramite `.gitattributes` esclude `outputs/`, che contiene esperimenti esplorativi storici. Non comprende ambiente virtuale, cache, impostazioni IDE, PDF locale degli appunti o file non versionati. Cache e metadati `egg-info` sono stati rimossi dal versionamento, non cancellati dall'ambiente locale.

La preparazione di commit e archivio è locale: non implica caricamento sul repository remoto o consegna al professore.
