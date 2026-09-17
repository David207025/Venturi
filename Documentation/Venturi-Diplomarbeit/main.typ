#import "template/template.typ": diplomarbeit

#show: diplomarbeit.with(
  title: json("meta.json").title,
  subtitle: json("meta.json").subtitle,
  authors: ("Valerie Niederreiter", "David Vacaroiu"),
  author-footer: "V. Niederreiter, D. Vacaroiu",
  class-name: "5AHEL",
  classof: "2026/2027",
  typeofwork: "Diplomarbeit",
  study: "Elektronik",
  department: "Abteilung für Elektronik und Technische Informatik",
  supervisor: json("meta.json").betreuer,
  submission-town: "Salzburg",
  submission-month: "April",
  submission-year: "2027",
)


#include "formal/title.typ"
#include "formal/declaration.typ"
#include "formal/diplomarbeit-doku.typ"

// Frontmatter Content
#include "content/ch-abstract.typ"
#include "content/ch-danksagung.typ"
#include "content/ch-abk-verzeichnis.typ"

// Table of Contents (\setcounter{tocdepth}{1})
#pagebreak()
#outline(indent: auto, depth: 2)

// Content Chapters
#include "content/ch-einleitung.typ"

// Appendix & Lists
#pagebreak()

#set align(center + horizon)
#heading(numbering: none, outlined: false)[Anhang]
#set align(left + top)

#outline(
  title: [Abbildungsverzeichnis],
  target: figure.where(kind: image),
)

#outline(
  title: [Tabellenverzeichnis],
  target: figure.where(kind: table),
)

#pagebreak()
#heading(numbering: none)[Quellenverzeichnis]

#v(0.3cm)
#heading(level: 2, numbering: none)[Quellenverzeichnis -- Valerie Niederreiter]
#bibliography(
  "references-biblatex-niev.bib",
  title: none,
  style: "ieee",
)

#v(0.5cm)
#heading(level: 2, numbering: none)[Quellenverzeichnis -- David Vacaroiu]
#bibliography(
  "references-biblatex-vacd.bib",
  title: none,
  style: "ieee",
)

//#include "content/ch-PCBDoku.typ"
#include "formal/protokoll.typ"
