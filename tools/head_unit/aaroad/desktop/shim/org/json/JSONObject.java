package org.json;

import java.util.LinkedHashMap;
import java.util.Map;

// A small stand-in for Android's org.json (the JDK has none): enough for Feed's datagrams.
public class JSONObject {
  public static final Object NULL = new Object() { public String toString() { return "null"; } };
  final Map<String, Object> map = new LinkedHashMap<String, Object>();

  public JSONObject() {}
  public JSONObject(String s) throws JSONException {
    Object o = new Parser(s).value();
    if (!(o instanceof JSONObject)) throw new JSONException("not an object");
    map.putAll(((JSONObject) o).map);
  }
  JSONObject(Map<String, Object> m) { map.putAll(m); }

  public JSONObject put(String k, Object v) throws JSONException { map.put(k, v == null ? NULL : v); return this; }
  public JSONObject put(String k, int v) throws JSONException { return put(k, (Object) Integer.valueOf(v)); }
  public JSONObject put(String k, long v) throws JSONException { return put(k, (Object) Long.valueOf(v)); }
  public JSONObject put(String k, double v) throws JSONException { return put(k, (Object) Double.valueOf(v)); }
  public JSONObject put(String k, boolean v) throws JSONException { return put(k, (Object) Boolean.valueOf(v)); }
  public boolean has(String k) { return map.containsKey(k); }
  public boolean isNull(String k) { Object o = map.get(k); return o == null || o == NULL; }
  public Object opt(String k) { return map.get(k); }

  private static double num(Object o, double d) {
    if (o instanceof Number) return ((Number) o).doubleValue();
    if (o instanceof String) { try { return Double.parseDouble((String) o); } catch (NumberFormatException e) { return d; } }
    return d;
  }
  public double optDouble(String k, double d) { Object o = map.get(k); return o == null || o == NULL ? d : num(o, d); }
  public double optDouble(String k) { return optDouble(k, Double.NaN); }
  public int optInt(String k, int d) { Object o = map.get(k); return o == null || o == NULL ? d : (int) num(o, d); }
  public int optInt(String k) { return optInt(k, 0); }
  public long optLong(String k, long d) { Object o = map.get(k); return o == null || o == NULL ? d : (long) num(o, d); }
  public long optLong(String k) { return optLong(k, 0); }
  public boolean optBoolean(String k, boolean d) {
    Object o = map.get(k);
    if (o instanceof Boolean) return (Boolean) o;
    if (o instanceof String) return Boolean.parseBoolean((String) o);
    return d;
  }
  public boolean optBoolean(String k) { return optBoolean(k, false); }
  public String optString(String k, String d) { Object o = map.get(k); return o == null || o == NULL ? d : o.toString(); }
  public String optString(String k) { return optString(k, ""); }
  public JSONObject optJSONObject(String k) { Object o = map.get(k); return o instanceof JSONObject ? (JSONObject) o : null; }
  public JSONArray optJSONArray(String k) { Object o = map.get(k); return o instanceof JSONArray ? (JSONArray) o : null; }

  private Object need(String k) throws JSONException {
    Object o = map.get(k);
    if (o == null || o == NULL) throw new JSONException("no " + k);
    return o;
  }
  public double getDouble(String k) throws JSONException { Object o = need(k); if (!(o instanceof Number)) throw new JSONException(k); return ((Number) o).doubleValue(); }
  public int getInt(String k) throws JSONException { return (int) getDouble(k); }
  public long getLong(String k) throws JSONException { return (long) getDouble(k); }
  public String getString(String k) throws JSONException { return need(k).toString(); }
  public JSONArray getJSONArray(String k) throws JSONException { Object o = need(k); if (!(o instanceof JSONArray)) throw new JSONException(k); return (JSONArray) o; }
  public JSONObject getJSONObject(String k) throws JSONException { Object o = need(k); if (!(o instanceof JSONObject)) throw new JSONException(k); return (JSONObject) o; }

  @Override
  public String toString() {
    StringBuilder b = new StringBuilder("{");
    for (Map.Entry<String, Object> e : map.entrySet()) {
      if (b.length() > 1) b.append(',');
      b.append(quote(e.getKey())).append(':').append(str(e.getValue()));
    }
    return b.append('}').toString();
  }
  static String str(Object o) { return o instanceof String ? quote((String) o) : String.valueOf(o); }
  static String quote(String s) {
    StringBuilder b = new StringBuilder("\"");
    for (char c : s.toCharArray()) {
      if (c == '"' || c == '\\') b.append('\\').append(c);
      else if (c < 32) b.append(String.format("\\u%04x", (int) c));
      else b.append(c);
    }
    return b.append('"').toString();
  }

  static final class Parser {
    final String s;
    int i;
    Parser(String s) { this.s = s; }
    void ws() { while (i < s.length() && Character.isWhitespace(s.charAt(i))) i++; }
    Object value() throws JSONException {
      ws();
      if (i >= s.length()) throw new JSONException("end");
      char c = s.charAt(i);
      if (c == '{') {
        i++;
        JSONObject o = new JSONObject();
        ws();
        if (s.charAt(i) == '}') { i++; return o; }
        while (true) {
          ws();
          String k = (String) value();
          ws();
          if (s.charAt(i++) != ':') throw new JSONException("colon");
          o.map.put(k, value());
          ws();
          char d = s.charAt(i++);
          if (d == '}') return o;
          if (d != ',') throw new JSONException("comma");
        }
      }
      if (c == '[') {
        i++;
        JSONArray a = new JSONArray();
        ws();
        if (s.charAt(i) == ']') { i++; return a; }
        while (true) {
          a.list.add(value());
          ws();
          char d = s.charAt(i++);
          if (d == ']') return a;
          if (d != ',') throw new JSONException("comma");
        }
      }
      if (c == '"') {
        i++;
        StringBuilder b = new StringBuilder();
        while (s.charAt(i) != '"') {
          char d = s.charAt(i++);
          if (d == '\\') {
            char e = s.charAt(i++);
            if (e == 'u') { b.append((char) Integer.parseInt(s.substring(i, i + 4), 16)); i += 4; }
            else b.append(e == 'n' ? '\n' : e == 't' ? '\t' : e == 'r' ? '\r' : e == 'b' ? '\b' : e == 'f' ? '\f' : e);
          } else b.append(d);
        }
        i++;
        return b.toString();
      }
      if (s.startsWith("true", i)) { i += 4; return Boolean.TRUE; }
      if (s.startsWith("false", i)) { i += 5; return Boolean.FALSE; }
      if (s.startsWith("null", i)) { i += 4; return NULL; }
      int j = i;
      while (j < s.length() && "+-0123456789.eE".indexOf(s.charAt(j)) >= 0) j++;
      String t = s.substring(i, j);
      i = j;
      try {
        if (t.indexOf('.') < 0 && t.indexOf('e') < 0 && t.indexOf('E') < 0) {
          long l = Long.parseLong(t);
          return l >= Integer.MIN_VALUE && l <= Integer.MAX_VALUE ? (Object) Integer.valueOf((int) l) : (Object) Long.valueOf(l);
        }
        return Double.valueOf(t);
      } catch (NumberFormatException e) {
        throw new JSONException("number " + t);
      }
    }
  }
}
