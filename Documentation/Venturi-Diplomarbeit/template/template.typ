#let diplomarbeit(
  title: "",
  subtitle: "",
  authors: (),
  author-footer: "",
  class-name: "",
  classof: "",
  typeofwork: "",
  study: "",
  department: "",
  supervisor: "",
  submission-town: "",
  submission-month: "",
  submission-year: "",
  body,
) = {
  // Page Geometry & KOMA-Script Header/Footer setup
  set page(
    paper: "a4",
    margin: (top: 2.5cm, bottom: 2.5cm, inside: 3cm, outside: 2.5cm),
    header: context {
      let page-num = counter(page).get().first()
      if page-num > 1 {
        let is-even = calc.even(page-num)
        let left-info = if is-even { classof } else { typeofwork }
        let right-info = if is-even { typeofwork } else { classof }

        grid(
          columns: (1fr, 1fr, 1fr),
          align: (left, center, right),
          text(size: 9pt)[#left-info], text(size: 9pt)[#title], text(size: 9pt)[#right-info],
        )
        v(-0.4em)
        line(length: 100%, stroke: 0.5pt + black)
      }
    },
    footer: context {
      let page-num = counter(page).get().first()
      let total-pages = counter(page).final().first()
      if page-num > 1 {
        let is-even = calc.even(page-num)
        let left-info = if is-even { "HTBLuVA -- " + study } else { author-footer + ", " + class-name }
        let right-info = if is-even { author-footer + ", " + class-name } else { "HTBLuVA -- " + study }

        line(length: 100%, stroke: 0.5pt + black)
        v(0.2em)
        grid(
          columns: (1fr, 1fr, 1fr),
          align: (left, center, right),
          text(size: 9pt)[#left-info], text(size: 9pt)[#page-num / #total-pages], text(size: 9pt)[#right-info],
        )
      }
    },
  )

  // Typography Settings (\myfontsize{12pt}, \mylinespread{1.5}, \myparskip{half})
  set text(
    font: "Times New Roman",
    size: 14pt,
    lang: "de",
    region: "AT",
  )

  set par(
    justify: true,
    leading: 0.75em,
    spacing: 0.8em,
  )

  // Chapter formatting (\RedeclareSectionCommand[beforeskip=0pt, afterskip=1cm]{chapter})
  set heading(numbering: "1.1")
  show heading.where(level: 1): it => {
    pagebreak(weak: true)
    v(0cm)
    text(size: 22pt, weight: "black")[#it.body]
    v(1cm)
  }

  show outline.entry.where(level: 1): set text(weight: "bold")

  body
}
