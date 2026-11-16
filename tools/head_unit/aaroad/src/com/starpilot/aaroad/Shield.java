package com.starpilot.aaroad;

import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

// Highway numbers and exit numbers in road text, for the badges Google Maps draws (Peter 2026-10-31: "for the exit
// can you put it in the green box like actual highway exit is annotated on Google Maps. Same with highway number in
// a white box"). The turn card, the street tag and the map labels all ask here, so they agree on what is a badge.
// Plain Java, no Android: test/ShieldTest.java runs it on the desktop.
//
// A route number is one of the forms Mapbox's banners, step refs and road tiles use, written the one way the badge
// shows it: "I-635" (I 635, I-635, IH 635, Interstate 635), "US 75" (US-75, U.S. 75, US Highway 75), "SH 121" (State
// Highway 121), "SR 1" (State Route 1), a state's own ("TX 121", "CA-1"), and Texas's "Loop 12", "Spur 366", "FM 1709",
// "RM 620", "Beltway 8", "CR 123". A letter stuck to the number stays ("I-35E"); a direction after a space does not
// ("I-635 E" is the badge "I-635" then "E"). Named roads stay text, tollways too ("Dallas North Tollway"), and so does
// a word with no number ("US Highway", "Interstate").
final class Shield {
  static final int TEXT = 0, EXIT = 1, REF = 2;

  // one piece of a line: words, an exit number ("34A") or a route number ("US 75")
  static final class Piece {
    final int kind;
    final String text;

    Piece(int kind, String text) {
      this.kind = kind;
      this.text = text;
    }

    @Override
    public String toString() {
      return (kind == EXIT ? "[exit " : kind == REF ? "[" : "") + text + (kind == TEXT ? "" : "]");
    }
  }

  private static final String NUM = "(\\d{1,4}[A-Z]?)", SEP = "[- \\u2011]";
  private static final String STATES = "AL|AK|AZ|AR|CA|CO|CT|DE|FL|GA|HI|ID|IL|IN|IA|KS|KY|LA|ME|MD|MA|MI|MN|MS|MO|MT|"
      + "NE|NV|NH|NJ|NM|NY|NC|ND|OH|OK|OR|PA|RI|SC|SD|TN|TX|UT|VT|VA|WA|WV|WI|WY";
  // groups in pairs: the kind as written, then the number
  private static final String REF_RE =
      "(Interstate|IH|I)" + SEP + "?(\\d{1,3}[A-Z]?)"
      + "|(U\\.?S\\.?(?: (?:Highway|Hwy|Route|Rte))?)" + SEP + "?(\\d{1,3}[A-Z]?)"
      + "|(State (?:Highway|Hwy|Route|Rte)|SH|SR)" + SEP + "?" + NUM
      + "|(" + STATES + ")" + SEP + NUM
      + "|(Loop|Spur|FM|RM|RR|CR|County Road|Farm(?: to |-to-)Market Road|Ranch(?: to |-to-)Market Road|Beltway)" + SEP + NUM;
  // a route number anywhere in a line, not part of a longer word
  private static final Pattern ANY_REF = Pattern.compile("(?<![\\w.\\u2011-])(?:" + REF_RE + ")(?![\\w\\u2011-])");
  // a whole name that is a route number, maybe with a direction after it
  private static final Pattern WHOLE = Pattern.compile("(?:" + REF_RE + ")(?: (?:N|S|E|W|NB|SB|EB|WB|North|South|East|West))?");
  // "Exit 34A", "Exit 19A:", "Exits 1A-B" at the start of an instruction
  private static final Pattern EXIT_RE = Pattern.compile("^(?:Exit|EXIT|exit)s? (\\d{1,4}[A-Za-z]?(?:-[A-Za-z0-9]{1,4})?)(?![\\w])[:,]?\\s*");

  // The route number s names, as the badge shows it ("I 635" -> "I-635"), or null if s is not one.
  static String ref(String s) {
    if (s == null) return null;
    Matcher m = WHOLE.matcher(s.trim());
    return m.matches() ? norm(m) : null;
  }

  // The first route number of a Mapbox ref ("US 75; TX 289", "I 635;US 75"), or null.
  static String firstRef(String refs) {
    if (refs == null) return null;
    for (String r : refs.split("[;/]")) {
      String n = ref(r);
      if (n != null) return n;
    }
    return null;
  }

  // The leading exit number of an instruction ("Exit 34 McDermott Drive" -> "34"), or null.
  static String exit(String text) {
    if (text == null) return null;
    Matcher m = EXIT_RE.matcher(text);
    return m.find() ? m.group(1).toUpperCase(java.util.Locale.US) : null;
  }

  // text cut into words, exit and route numbers, in order; the words between badges trimmed, empty ones left out.
  static List<Piece> split(String text) {
    List<Piece> out = new ArrayList<Piece>();
    if (text == null) return out;
    Matcher e = EXIT_RE.matcher(text);
    int from = 0;
    if (e.find()) {
      out.add(new Piece(EXIT, e.group(1).toUpperCase(java.util.Locale.US)));
      from = e.end();
    }
    Matcher m = ANY_REF.matcher(text);
    m.region(from, text.length());
    while (m.find()) {
      words(out, text.substring(from, m.start()));
      out.add(new Piece(REF, norm(m)));
      from = m.end();
    }
    words(out, text.substring(from));
    return out;
  }

  private static void words(List<Piece> out, String s) {
    s = s.trim();
    if (!s.isEmpty()) out.add(new Piece(TEXT, s));
  }

  private static String norm(Matcher m) {
    for (int g = 1; g + 1 <= m.groupCount(); g += 2) {
      String kind = m.group(g);
      if (kind == null) continue;
      String n = m.group(g + 1);
      switch (g) {
        case 1: return "I-" + n;
        case 3: return "US " + n;
        case 5: return (kind.startsWith("State R") || kind.equals("SR") ? "SR " : "SH ") + n;
        case 7: return kind + " " + n;
        default:
          String k = kind.startsWith("County") ? "CR" : kind.startsWith("Farm") ? "FM" : kind.startsWith("Ranch") ? "RM" : kind;
          return k + " " + n;
      }
    }
    return m.group();
  }

  private Shield() {}
}
