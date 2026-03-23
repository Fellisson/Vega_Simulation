# Vega_Simulation

Depot MATLAB centre uniquement sur deux simulations 3D sans PID :

- `modele_final_3D` : modele generique final
- `modele_vega` : modele inspire du lanceur Vega-C

## Objectif

Ce depot rassemble une base minimale pour etudier et comparer :

- un modele academique generique de vehicule propulse en 3D
- un modele calibre sur des ordres de grandeur inspires de Vega-C

L'objectif est de fournir un projet simple, lisible et directement reutilisable pour la simulation, l'analyse et la presentation des resultats.

## Structure

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
`- README.md
```

## Scripts principaux

### `modele_final_3D.m`

Modele 3D generique sans PID.

- trajectoire propulsee simplifiee
- masse variable
- trainee aerodynamique
- attitude en 3D
- export image, video et log

### `modele_vega.m`

Modele 3D sans PID inspire de Vega-C.

- vehicule multi-etages simplifie
- parametres de masse, poussee et geometrie inspires du reel
- ascension 3D avec altitude cible
- export image, video et log

## Demarrage rapide

Depuis MATLAB :

```matlab
run('setup_paths.m')
modele_final_3D('preview')
modele_vega('preview')
```

Pour un calcul rapide sans rendu :

```matlab
run('setup_paths.m')
modele_final_3D('no_render')
modele_vega('no_render')
```

## Sorties generees

Les resultats sont enregistres dans `out/` :

- `out/images/` : figures PNG
- `out/logs/` : resumes textuels et presentations
- `out/videos/` : animations MP4

## Resultats de reference

### Modele final 3D

- temps de vol : `19.00 s`
- portee horizontale : `2306.52 m`
- altitude maximale : `12234.30 m`

### Modele Vega

- temps de vol : `159.25 s`
- portee horizontale : `62334.92 m`
- altitude maximale : `250091.76 m`

## Auteur

Felix Nimy
