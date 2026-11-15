package org.json;

import java.util.ArrayList;
import java.util.List;

public class JSONArray {
  final List<Object> list = new ArrayList<Object>();

  public JSONArray() {}
  public int length() { return list.size(); }
  public JSONArray put(Object o) { list.add(o == null ? JSONObject.NULL : o); return this; }
  public Object opt(int i) { return i >= 0 && i < list.size() ? list.get(i) : null; }
  private static double num(Object o, double d) {
    if (o instanceof Number) return ((Number) o).doubleValue();
    if (o instanceof String) { try { return Double.parseDouble((String) o); } catch (NumberFormatException e) { return d; } }
    return d;
  }
  public int optInt(int i, int d) { Object o = opt(i); return o == null ? d : (int) num(o, d); }
  public int optInt(int i) { return optInt(i, 0); }
  public JSONObject optJSONObject(int i) { Object o = opt(i); return o instanceof JSONObject ? (JSONObject) o : null; }
  public JSONArray optJSONArray(int i) { Object o = opt(i); return o instanceof JSONArray ? (JSONArray) o : null; }
  private Object need(int i) throws JSONException { Object o = opt(i); if (o == null) throw new JSONException("index " + i); return o; }
  public double getDouble(int i) throws JSONException { Object o = need(i); if (!(o instanceof Number)) throw new JSONException("nan"); return ((Number) o).doubleValue(); }
  public JSONArray getJSONArray(int i) throws JSONException { Object o = need(i); if (!(o instanceof JSONArray)) throw new JSONException("array"); return (JSONArray) o; }
  @Override
  public String toString() {
    StringBuilder b = new StringBuilder("[");
    for (Object o : list) {
      if (b.length() > 1) b.append(',');
      b.append(JSONObject.str(o));
    }
    return b.append(']').toString();
  }
}
