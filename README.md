# Vega_Simulation

**Resume scientifique**  
Ce depot presente deux simulations 3D sans PID sous MATLAB : un modele final generique et un modele inspire du lanceur Vega-C. Il permet d'analyser la dynamique d'un vehicule propulse, jusqu'au deploiement simplifie d'un satellite et a la retombee du lanceur.

Simulation MATLAB de deux modeles 3D sans PID pour l'etude de la dynamique d'un vehicule propulse :

- `modele_final_3D` : modele generique final
- `modele_vega` : modele inspire du lanceur Vega-C

Ce depot a pour objectif de fournir une base de simulation compacte, lisible et reproductible pour l'analyse de trajectoires 3D avec poussee, masse variable, trainee aerodynamique et attitude.

## Apercu

Le projet rassemble deux niveaux de modelisation complementaires :

- un modele academique generique, utile pour l'analyse methodologique
- un modele calibre sur des ordres de grandeur inspires de Vega-C, utile pour une interpretation plus proche d'un lanceur spatial reel

Les deux scripts produisent automatiquement des figures, des fichiers de log et, selon le mode choisi, des videos exportees dans le dossier `out/`.

## Apercu visuel

### Modele Vega avec separation de satellite

![Modele Vega](out/images/modele_vega.png)

## Contenu du depot

```text
Vega_Simulation/
|- src/
|  `- controle_pid/
|     |- modele_final_3D.m
|     `- modele_vega.m
|- out/
|  |- images/
|  |- logs/
|  `- videos/
|- setup_paths.m
|- .gitignore
`- README.md
```

## Modeles inclus

### `modele_final_3D`

Modele 3D generique sans PID destine a servir de reference academique.

Principales caracteristiques :

- trajectoire propulsee simplifiee
- masse variable
- trainee aerodynamique
- dynamique d'attitude en 3D
- export des resultats en image, log et video

### `modele_vega`

Modele 3D sans PID inspire du lanceur Vega-C.

Principales caracteristiques :

- vehicule multi-etages simplifie
- parametres de masse, geometrie et poussee inspires du reel
- profil d'ascension 3D avec separation de satellite
- retombee balistique du lanceur apres largage
- poursuite de la trajectoire du satellite en espace apres l'impact du lanceur
- export des resultats en image, log et video

## Prerequis

- MATLAB
- acces aux fonctions standards de calcul numerique et de visualisation

Le projet ne depend d'aucune toolbox specifique supplementaire au-dela de l'environnement MATLAB utilise pour les scripts et les exportations.

## Demarrage rapide

Depuis MATLAB :

```matlab
run('setup_paths.m')
```

Exemples de lancement :

```matlab
modele_final_3D('preview')
modele_vega('preview')
```

Pour un calcul rapide sans rendu graphique :

```matlab
modele_final_3D('no_render')
modele_vega('no_render')
```

Pour generer une video :

```matlab
modele_final_3D('video')
modele_vega('video')
```

## Modes disponibles

Les deux scripts acceptent les modes suivants :

- `debug`
- `no_render`
- `preview`
- `video`
- `ultra_light`
- `full`

Ces modes permettent d'ajuster le temps de simulation, le niveau de rendu et l'export des videos.

## Resultats generes

Les sorties sont enregistrees automatiquement dans `out/` :

- `out/images/` : figures PNG
- `out/logs/` : resumes textuels et fichiers de presentation
- `out/videos/` : animations MP4

Exemples de fichiers produits :

- `out/images/modele_final_3D.png`
- `out/images/modele_vega.png`
- `out/logs/modele_final_3D.txt`
- `out/logs/modele_vega.txt`
- `out/videos/modele_final_3D.mp4`
- `out/videos/modele_vega.mp4`

## Resultats de reference

### Modele final 3D

- temps de vol : `19.00 s`
- portee horizontale : `2306.52 m`
- altitude maximale : `12234.30 m`
- fin de simulation : `target_altitude_reached`

### Modele Vega

- temps de vol : `362.50 s`
- portee horizontale : `155217.61 m`
- altitude maximale : `250091.76 m`
- fin de simulation : `satellite_deployed_then_ground_impact`
- separation du satellite : `159.25 s`
- altitude de separation : `250091.76 m`
- satellite poursuivi visuellement en espace apres la retombee du lanceur

## Portee scientifique

Le depot permet de comparer :

- une modelisation generique de vehicule propulse en 3D
- une adaptation appliquee a un cas inspire du secteur spatial
- un scenario de deploiement simplifie de satellite avec retombee du lanceur

Il constitue ainsi une base utile pour :

- l'analyse academique de la dynamique du vol
- la production de figures et de supports de presentation
- l'illustration d'un passage progressif d'un modele generique a un modele plus realiste

## Limites

Les deux modeles restent volontairement simplifies :

- pas de commande PID active dans les versions retenues
- atmosphere simplifiee
- trainee basee sur un coefficient constant
- poussees modelisees par valeurs moyennes
- absence de mecanique orbitale detaillee

Les resultats doivent donc etre interpretes comme des ordres de grandeur scientifiquement coherents, et non comme des predictions industrielles exactes.

## Auteur

Felix Nimy
