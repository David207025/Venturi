// Convenience macros and helpers for Typst

#let acro(term) = smallcaps(term)

#let rotate-page(content) = {
  page(flipped: true)[#content]
}
