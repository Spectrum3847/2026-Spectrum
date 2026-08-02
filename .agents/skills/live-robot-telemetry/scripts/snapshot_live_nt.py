#!/usr/bin/env python3
"""Read one timestamped snapshot of selected live NT4 topics."""

from __future__ import annotations

import argparse
from nt_live_common import compile_and_run


JAVA_SOURCE = r'''
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.Topic;

public class SnapshotLiveNt {
  private static String jsonEscape(String value) {
    return value.replace("\\", "\\\\").replace("\"", "\\\"");
  }

  private static boolean waitConnected(NetworkTableInstance inst, double timeoutSeconds) throws Exception {
    long deadline = System.nanoTime() + (long)(timeoutSeconds * 1_000_000_000L);
    while (System.nanoTime() < deadline) {
      if (inst.isConnected()) {
        return true;
      }
      Thread.sleep(50);
    }
    return inst.isConnected();
  }

  private static String valueToText(NetworkTableValue value) {
    if (value == null || !value.isValid()) return "<unset>";
    switch (value.getType()) {
      case kBoolean: return Boolean.toString(value.getBoolean());
      case kDouble: return Double.toString(value.getDouble());
      case kInteger: return Long.toString(value.getInteger());
      case kString: return value.getString();
      case kBooleanArray: return java.util.Arrays.toString(value.getBooleanArray());
      case kDoubleArray: return java.util.Arrays.toString(value.getDoubleArray());
      case kIntegerArray: return java.util.Arrays.toString(value.getIntegerArray());
      case kStringArray: return java.util.Arrays.toString(value.getStringArray());
      case kRaw: return "<raw " + value.getRaw().length + " bytes>";
      default: return value.toString();
    }
  }

  private static String valueToJson(NetworkTableValue value) {
    if (value == null || !value.isValid()) return "{\"valid\":false}";
    return "{\"valid\":true,\"type\":\"" + jsonEscape(value.getType().getValueStr()) + "\",\"value\":\""
        + jsonEscape(valueToText(value)) + "\"}";
  }

  public static void main(String[] args) throws Exception {
    String host = args[0];
    int port = Integer.parseInt(args[1]);
    double timeout = Double.parseDouble(args[2]);
    boolean json = Boolean.parseBoolean(args[3]);
    String[] topicNames = new String[args.length - 4];
    System.arraycopy(args, 4, topicNames, 0, topicNames.length);

    NetworkTableInstance inst = NetworkTableInstance.create();
    inst.startClient4("codex-live-snapshot");
    inst.setServer(host, port);
    if (!waitConnected(inst, timeout)) {
      System.err.println("Timed out waiting for NT4 connection to " + host + ":" + port);
      inst.close();
      System.exit(2);
    }

    GenericSubscriber[] subscribers = new GenericSubscriber[topicNames.length];
    for (int i = 0; i < topicNames.length; i++) {
      Topic topic = inst.getTopic(topicNames[i]);
      subscribers[i] = topic.genericSubscribe("", PubSubOption.pollStorage(20));
    }
    Thread.sleep(250);

    double timestamp = System.currentTimeMillis() / 1000.0;
    if (json) {
      System.out.println("{\"host\":\"" + jsonEscape(host) + "\",\"port\":" + port + ",\"timestamp\":" + timestamp + ",\"values\":{");
      for (int i = 0; i < topicNames.length; i++) {
        NetworkTableValue value = subscribers[i].get();
        System.out.print("  \"" + jsonEscape(topicNames[i]) + "\":" + valueToJson(value));
        System.out.println(i + 1 == topicNames.length ? "" : ",");
      }
      System.out.println("}}");
    } else {
      System.out.println("Connected to " + host + ":" + port);
      System.out.printf("Timestamp: %.3f%n", timestamp);
      for (int i = 0; i < topicNames.length; i++) {
        System.out.println(topicNames[i] + "\t" + valueToText(subscribers[i].get()));
      }
    }

    for (GenericSubscriber subscriber : subscribers) {
      subscriber.close();
    }
    inst.close();
  }
}
'''


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="10.80.44.2", help="Robot IP or hostname")
    parser.add_argument("--port", type=int, default=5810, help="NT4 port")
    parser.add_argument("--timeout", type=float, default=5.0, help="Connection timeout in seconds")
    parser.add_argument("--topic", action="append", required=True, help="Topic to read; repeatable")
    parser.add_argument("--json", action="store_true", help="Print JSON")
    args = parser.parse_args()

    raise SystemExit(
        compile_and_run(
            JAVA_SOURCE,
            "SnapshotLiveNt",
            [args.host, str(args.port), str(args.timeout), str(args.json).lower(), *args.topic],
        )
    )


if __name__ == "__main__":
    main()
