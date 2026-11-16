# Workflow del progetto

## 1. Obiettivo

Il progetto simula un sistema multi-agente planare composto da $N\geq3$ robot (default 3), con dinamica a singolo integratore:

$$
\dot p_i=u_i,
\qquad p_i\in\mathbb R^2.
$$

Gli obiettivi sono:

1. raggiungere e mantenere una formazione poligonale regolare rigida (triangolo equilatero per $N=3$);
2. navigare verso un goal comune;
3. evitare ostacoli statici e mobili;
4. analizzare i limiti degli Artificial Potential Fields, inclusi minimi locali.

---

## 2. Modello e simulatore

Sono stati implementati:

- agenti circolari con posizione, raggio, velocità massima e saturazione del comando;
- ostacoli circolari statici o mobili;
- integrazione di Eulero:

$$
p_i[k+1]=p_i[k]+\Delta t\,\bar u_i[k],
$$

$$
q_j[k+1]=q_j[k]+\Delta t\,v_j;
$$

- aggiornamento sincrono: tutti i controlli sono calcolati sullo stato al tempo $k$, poi agenti e ostacoli sono aggiornati simultaneamente.

La saturazione evita comandi di velocità irrealistici:

$$
\bar u_i=
\begin{cases}
u_i, & \lVert u_i\rVert\leq u_{\max,i},\\
u_{\max,i}\dfrac{u_i}{\lVert u_i\rVert}, & \lVert u_i\rVert\gt u_{\max,i}.
\end{cases}
$$

---

## 3. Controllo della formazione

La formazione è un poligono regolare rigido rappresentato dal grafo completo $K_N$, con $N\geq3$. Il triangolo originale corrisponde a $N=3$. Il numero di agenti è configurabile in tutti gli esperimenti con `--agent-count`.

Gli archi impongono le distanze desiderate:

$$
\lVert p_i-p_j\rVert=d_{ij}.
$$

Per un triangolo equilatero di lato $2\,\mathrm m$:

$$
d_{01}=d_{02}=d_{12}=2.
$$

La stessa funzione `TriangularFormation.equilateral(agent_ids, side_length=L)` genera quadrati, pentagoni, esagoni e altri poligoni regolari a partire dalla sola lunghezza $L$ del lato perimetrale. Il nome della classe è mantenuto per compatibilità. Le diagonali sono vincolate alle corde del poligono, non alla lunghezza del lato:

$$
d_{ij}=L\frac{\sin(\pi\Delta_{ij}/N)}{\sin(\pi/N)},
\qquad
\Delta_{ij}=\min(\lvert i-j\rvert,N-\lvert i-j\rvert).
$$

Gli indici qui rappresentano l'ordine ciclico dei vertici. I soli lati uguali non garantirebbero la rigidità per $N\gt3$.

Il potenziale di formazione è quello trattato negli appunti:

$$
V_f(p)=\frac{k_f}{4}
\sum_{(i,j)\in E}
\left(\lVert p_i-p_j\rVert^2-d_{ij}^2\right)^2.
$$

Il controllo distribuito è:

$$
u_{f,i}=
-k_f\sum_{j\in\mathcal N_i}
\left(\lVert p_i-p_j\rVert^2-d_{ij}^2\right)
(p_i-p_j).
$$

Ogni agente usa quindi soltanto la propria posizione e quelle dei vicini nel grafo. Negli esperimenti il guadagno viene scalato come

$$
k_f(N)=\frac{3}{N}\left(\frac{\sin(\pi/N)}{\sin(\pi/3)}\right)^2.
$$

Vale 1 per tre agenti; la scalatura limita l'aumento della rigidezza numerica, ma non garantisce stabilità per ogni passo o numerosità. Per $N\gt3$ le posizioni iniziali casuali vengono ordinate angolarmente attorno al baricentro. Per $N\geq6$ il semilato dello spazio di estrazione aumenta da 3 m a $3\sqrt{N/3}$ m.

---

## 4. Navigazione verso il goal

Il goal viene usato solo dopo che la formazione è stata raggiunta entro una tolleranza:

$$
\max_{(i,j)\in E}
\lvert
\lVert p_i-p_j\rVert-d_{ij}
\rvert
\leq \varepsilon_f.
$$

Il goal è estratto casualmente su una circonferenza di raggio fissato rispetto al baricentro:

$$
c=\frac{1}{N}\sum_{i=1}^Np_i,
$$

$$
g=c+D_g
\begin{bmatrix}
\cos\theta\\
\sin\theta
\end{bmatrix}.
$$

Inizialmente era presente un potenziale quadratico sul goal. Successivamente è stato sostituito con una funzione di Huber scalata:

$$
\mathcal H_{\delta_g}(e)=
\begin{cases}
\dfrac{1}{2}e^2, & e\leq\delta_g,\\
\delta_g\left(e-\dfrac{\delta_g}{2}\right), & e\gt\delta_g,
\end{cases}
$$

$$
V_g(p)=\frac{k_gN}{\delta_g}
\mathcal H_{\delta_g}(\lVert c-g\rVert).
$$

Questa scelta rende il modulo della velocità di navigazione costante lontano dal goal:

$$
u_{g,i}=k_g\frac{g-c}{\lVert g-c\rVert},
\qquad
\lVert c-g\rVert\gt\delta_g.
$$

Quindi il MAS non rallenta progressivamente mentre si avvicina al goal, come accadrebbe con il potenziale quadratico puro.

---

## 5. Evitamento ostacoli

Per ogni coppia agente-ostacolo viene calcolata la clearance tra i bordi:

$$
\rho_{ij}=
\lVert p_i-q_j\rVert-r_i-R_j.
$$

Il potenziale repulsivo usato è:

$$
V_{o,ij}=
\begin{cases}
\dfrac{k_o}{2}
\left(
\dfrac{1}{\rho_{ij}}
-
\dfrac{1}{\rho_0}
\right)^2,
& 0\lt\rho_{ij}\lt\rho_0,\\
0, & \rho_{ij}\geq\rho_0.
\end{cases}
$$

Il controllo è il negativo del gradiente:

$$
u_{o,i}=
\sum_{j\in\mathcal O_i}
k_o
\left(
\frac{1}{\rho_{ij}}
-
\frac{1}{\rho_0}
\right)
\frac{p_i-q_j}
{\rho_{ij}^2\lVert p_i-q_j\rVert}.
$$

Parametri principali:

- raggio agente: $0.15\,\mathrm m$;
- raggio ostacolo: $0.45\,\mathrm m$;
- distanza di influenza: $\rho_0=2.0\,\mathrm m$;
- attivazione del campo a distanza tra centri inferiore a:

$$
0.15+0.45+2.0=2.6\,\mathrm m.
$$

È presente una regolarizzazione numerica:

$$
\widetilde \rho_{ij}=\max(\rho_{ij},\rho_{\min}),
$$

con $\rho_{\min}=0.05\,\mathrm m$, per evitare singolarità.

Per gli **ostacoli mobili** (`PredictiveObstacleAvoidanceController`), conoscendo la velocità costante $v_j$, vengono predette le posizioni future lungo l'orizzonte $T_p$ su $M_p$ passi ($\tau_m = \frac{m}{M_p}T_p$, $m=0,\dots,M_p$):

$$
\widehat q_{j,m}(t) = q_j(t) + \tau_m v_j.
$$

Per ogni ostacolo si valutano la distanza attuale ($m=0$) e quelle future ($m=1,\dots,M_p$), selezionando la **distanza minore**:

$$
m_{ij}^\star = \arg\min_{m\in\{0,\dots,M_p\}} \lVert p_i - \widehat q_{j,m}(t)\rVert,
\qquad
q_{ij}^\star = \widehat q_{j,m_{ij}^\star}(t),
\qquad
\rho_{ij}^\star = \lVert p_i - q_{ij}^\star\rVert - r_i - R_j,
$$

e calcolando potenziale e forza repulsiva rispetto a $q_{ij}^\star$ e $\rho_{ij}^\star$.

L'agente viene mantenuto nella sua **posizione attuale** durante questa previsione: non è un modello del moto relativo agente–ostacolo. Il minimo è campionato, non continuo. Ai pareggi viene selezionato il primo campione temporale; il potenziale può non essere differenziabile e il comando può avere discontinuità. Il clipping della clearance mantiene finito il comando ma non assicura l'assenza di collisioni.

---

## 6. Comando totale

Prima del raggiungimento della formazione:

$$
u_i=u_{f,i}+u_{o,i}.
$$

Dopo l'attivazione della navigazione:

$$
u_i=u_{f,i}+u_{o,i}+u_{g,i}.
$$

Il termine di avoidance resta sempre attivo, anche nella fase iniziale, per ragioni di sicurezza.

---

## 7. Esperimenti sviluppati

### A. Formazione senza goal né ostacoli

- agenti inizializzati casualmente;
- raggiungimento del triangolo equilatero;
- verifica della convergenza del potenziale di formazione.

```bash
.venv/bin/python main.py --experiment formation --seed 42
```

### B. Formazione e goal

- gli agenti formano il triangolo;
- solo dopo si muovono verso un goal casuale;
- il goal è posto a distanza significativa dal baricentro.

```bash
.venv/bin/python main.py --experiment formation-goal --seed 42
```

### C. Ostacoli statici

- da 1 a 5 ostacoli fissi;
- ostacoli disposti lungo il corridoio di navigazione;
- analisi delle deviazioni, deformazioni e minimi locali.

```bash
.venv/bin/python main.py \
  --experiment static-obstacles \
  --obstacle-count 4 \
  --seed 42
```

Questo esperimento ha mostrato che il MAS può rimanere bloccato in un equilibrio indesiderato, in cui:

$$
u_{f,i}+u_{o,i}+u_{g,i}\approx 0,
$$

pur non avendo raggiunto il goal.

### D. Formazione perturbata da ostacoli mobili

- nessun goal;
- formazione inizialmente già corretta;
- ostacoli mobili casuali su traiettorie rettilinee;
- studio della deformazione della formazione causata dall'avoidance.

```bash
.venv/bin/python main.py \
  --experiment moving-obstacles \
  --moving-obstacle-count 2 \
  --seed 42
```

### E. Navigazione con ostacoli mobili casuali

È lo scenario più completo:

- condizioni iniziali casuali;
- formazione triangolare;
- goal casuale;
- ostacoli mobili;
- traiettorie rettilinee casuali progettate per attraversare il corridoio del MAS;
- grafici, potenziali e visualizzazione interattiva.

```bash
.venv/bin/python main.py \
  --experiment dynamic-navigation \
  --moving-obstacle-count 2 \
  --seed 42
```

---

## 8. Struttura del codice

```text
.
├── main.py
├── README.md
├── pyproject.toml
├── src/
│   └── mas_formation/
│       ├── models/
│       │   ├── agent.py
│       │   ├── obstacle.py
│       │   ├── environment.py
│       │   └── formation.py
│       ├── controllers/
│       │   ├── formation.py
│       │   ├── navigation.py
│       │   ├── obstacle_avoidance.py
│       │   └── mission.py
│       └── experiments/
│           ├── basic_formation.py
│           ├── html_viewer.py
│           └── compare_avoidance.py
├── scripts/
│   └── verify_clean_environment.py
├── tests/
├── docs/
│   ├── results/
│   └── interactive/
└── outputs/
```

### `models/`

- `Agent`: dinamica a singolo integratore e saturazione;
- `DynamicObstacle`: ostacolo circolare statico o mobile;
- `Environment`: aggiornamento sincrono della simulazione;
- `TriangularFormation`: nome storico del grafo $K_N$; costruzione di poligoni regolari, distanze desiderate e verifica degli errori.

### `controllers/`

- `FormationController`: controllo distribuito basato sul potenziale $V_f$;
- `GoalNavigationController`: navigazione del baricentro con Huber;
- `PositionBasedObstacleAvoidanceController`: avoidance APF basato sulla posizione istantanea;
- `PredictiveObstacleAvoidanceController`: avoidance APF predittivo che considera la distanza minima tra posizione attuale e posizioni future dell'ostacolo lungo l'orizzonte $T_p$;
- `StaticObstacleAvoidanceController`: variante che accetta solo ostacoli fissi;
- `FormationThenGoalController`: supervisore della sequenza formazione $\rightarrow$ navigazione;
- `FormationWithObstacleAvoidanceController`: formazione più avoidance, senza goal.

### `experiments/`

`basic_formation.py` contiene:

- definizione degli scenari;
- generazione casuale di posizioni, goal e ostacoli;
- raccolta delle storie temporali;
- creazione dei grafici;
- slider Matplotlib.

`html_viewer.py` esporta una pagina HTML autonoma con:

- slider temporale;
- play/pausa;
- scelta della velocità;
- traiettorie e stato istantaneo;
- valori dei potenziali.

`compare_avoidance.py` esegue le stesse condizioni iniziali con avoidance reattivo e predittivo e salva metriche JSON/CSV, traiettorie NPZ, grafici e un report Markdown. Il [confronto salvato](results/avoidance_comparison/report.md) include sia benefici sia compromessi, senza generalizzare il risultato di un singolo seed.

### `main.py`

È l'interfaccia CLI. Gestisce:

- scelta dell'esperimento;
- seed;
- durata;
- numero di agenti ($N\geq3$) e di ostacoli;
- orizzonte e campioni futuri della predizione;
- salvataggio dei grafici;
- export HTML;
- visualizzazione interattiva.

---

## 9. Visualizzazione e documentazione

Sono disponibili:

- grafici separati di traiettorie e potenziali;
- asse logaritmico per visualizzare $V_f$, $V_g$ e $V_o$;
- slider Matplotlib locale;
- export HTML in:

```text
docs/interactive/dynamic_navigation.html
```

La documentazione `README.md` contiene:

- formule;
- risultati sperimentali;
- figure;
- interpretazione dei potenziali;
- limiti;
- istruzioni di esecuzione.

---

## 10. Evitamento predittivo e limiti del modello

Per gli ostacoli mobili è stato implementato il controllo **predittivo** concordato a ricevimento:

$$
\widehat q_{j,m}(t)=q_j(t)+\tau_m v_j,
\qquad
\tau_m=\frac{m}{M_p}T_p,
\qquad
m=0,1,\dots,M_p,
$$

che valuta la distanza attuale e quelle future selezionando per ogni ostacolo quella minima $\rho_{ij}^\star = \min_m (\lVert p_i - \widehat q_{j,m}(t)\rVert - r_i - R_j)$. Ponendo $T_p = 0$ si recupera la baseline puramente reattiva.

Limiti strutturali del modello:

- nessuna garanzia formale di collision avoidance;
- possibili minimi locali degli APF;
- nessun avoidance esplicito tra agenti;
- ostacoli a velocità costante su traiettorie rettilinee;
- controllo di navigazione basato sul baricentro globale, quindi non completamente distribuito;
- saturazione dei comandi che può alterare le proprietà teoriche di gradiente;
- mancata previsione del moto degli agenti;
- minimo delle distanze campionato e possibili discontinuità del controllo ai pareggi.

Questi limiti degli APF vanno dichiarati nella relazione: non impediscono la conclusione dell'implementazione ma delimitano il significato dei risultati sperimentali.

## 11. Consegna e riproducibilità

```bash
# Confronto salvato, stessi seed e condizioni iniziali
.venv/bin/python -m mas_formation.experiments.compare_avoidance

# Poligoni con più agenti
.venv/bin/python main.py --experiment dynamic-navigation --agent-count 4 --seed 42 --duration 30 --interactive
.venv/bin/python main.py --experiment formation --agent-count 5 --seed 42 --interactive
.venv/bin/python main.py --experiment formation-goal --agent-count 6 --seed 42 --interactive

# Build, installazione e test in un ambiente temporaneo isolato
python3 scripts/verify_clean_environment.py
```

Per l'esito effettivo della verifica e le versioni usate, consultare [VERIFICA_FINALE.md](VERIFICA_FINALE.md).
