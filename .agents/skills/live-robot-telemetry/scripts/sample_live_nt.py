#!/usr/bin/env python3
"""Sample selected live NT4 topics for a bounded practice window."""

from __future__ import annotations

import argparse
from nt_live_common import compile_and_run


JAVA_SOURCE = r'''
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.Topic;
import java.util.ArrayList;
import java.util.List;

public class SampleLiveNt {
  static class Summary {
    int validSamples = 0;
    String first = "<unset>";
    String latest = "<unset>";
    double min = Double.POSITIVE_INFINITY;
    double max = Double.NEGATIVE_INFINITY;
    boolean numeric = true;

    void add(NetworkTableValue value) {
      if (value == null || !value.isValid()) {
        return;
      }
      String text = valueToText(value);
      if (validSamples == 0) {
        first = text;
      }
      latest = text;
      validSamples++;
      if (value.isDouble()) {
        double number = value.getDouble();
        min = Math.min(min, number);
        max = Math.max(max, number);
      } else if (value.isInteger()) {
        double number = value.getInteger();
        min = Math.min(min, number);
        max = Math.max(max, number);
      } else if (value.isBoolean()) {
        double number = value.getBoolean() ? 1.0 : 0.0;
        min = Math.min(min, number);
        max = Math.max(max, number);
      } else {
        numeric = false;
      }
    }
  }

  static class SampleRow {
    final double timestamp;
    final String[] values;

    SampleRow(double timestamp, String[] values) {
      this.timestamp = timestamp;
      this.values = values;
    }
  }

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

  private static void printJson(String host, int port, double duration, double period, String[] topicNames, Summary[] summaries, List<SampleRow> spikes) {
    System.out.println("{\"host\":\"" + jsonEscape(host) + "\",\"port\":" + port
        + ",\"duration\":" + duration + ",\"period\":" + period + ",\"topics\":{");
    for (int i = 0; i < topicNames.length; i++) {
      Summary s = summaries[i];
      System.out.print("  \"" + jsonEscape(topicNames[i]) + "\":{\"validSamples\":" + s.validSamples
          + ",\"first\":\"" + jsonEscape(s.first) + "\",\"latest\":\"" + jsonEscape(s.latest) + "\"");
      if (s.numeric && s.validSamples > 0) {
        System.out.print(",\"min\":" + s.min + ",\"max\":" + s.max);
      }
      System.out.print("}");
      System.out.println(i + 1 == topicNames.length ? "" : ",");
    }
    System.out.println("},\"spikes\":[");
    for (int i = 0; i < spikes.size(); i++) {
      SampleRow spike = spikes.get(i);
      System.out.print("  {\"timestamp\":" + spike.timestamp + ",\"values\":{");
      for (int topic = 0; topic < topicNames.length; topic++) {
        System.out.print("\"" + jsonEscape(topicNames[topic]) + "\":\"" + jsonEscape(spike.values[topic]) + "\"");
        if (topic + 1 < topicNames.length) {
          System.out.print(",");
        }
      }
      System.out.print("}}");
      System.out.println(i + 1 == spikes.size() ? "" : ",");
    }
    System.out.println("]}");
  }

  public static void main(String[] args) throws Exception {
    String host = args[0];
    int port = Integer.parseInt(args[1]);
    double timeout = Double.parseDouble(args[2]);
    double duration = Double.parseDouble(args[3]);
    double period = Double.parseDouble(args[4]);
    boolean json = Boolean.parseBoolean(args[5]);
    double spikeThreshold = Double.parseDouble(args[6]);
    String spikeTopic = args[7];
    boolean waitForEnabled = Boolean.parseBoolean(args[8]);
    boolean stopOnDisabled = Boolean.parseBoolean(args[9]);
    String enabledTopic = args[10];
    String[] topicNames = new String[args.length - 11];
    System.arraycopy(args, 11, topicNames, 0, topicNames.length);

    NetworkTableInstance inst = NetworkTableInstance.create();
    inst.startClient4("codex-live-sampler");
    inst.setServer(host, port);
    if (!waitConnected(inst, timeout)) {
      System.err.println("Timed out waiting for NT4 connection to " + host + ":" + port);
      inst.close();
      System.exit(2);
    }

    GenericSubscriber[] subscribers = new GenericSubscriber[topicNames.length];
    Summary[] summaries = new Summary[topicNames.length];
    for (int i = 0; i < topicNames.length; i++) {
      Topic topic = inst.getTopic(topicNames[i]);
      subscribers[i] = topic.genericSubscribe("", PubSubOption.pollStorage(1000));
      summaries[i] = new Summary();
    }

    long end = System.nanoTime() + (long)(duration * 1_000_000_000L);
    long sleepMillis = Math.max(10L, (long)(period * 1000.0));
    int spikeIndex = -1;
    int enabledIndex = -1;
    if (!spikeTopic.isEmpty()) {
      for (int i = 0; i < topicNames.length; i++) {
        if (topicNames[i].equals(spikeTopic)) {
          spikeIndex = i;
          break;
        }
      }
      if (spikeIndex < 0) {
        System.err.println("--spike-topic must also be provided as a --topic: " + spikeTopic);
        inst.close();
        System.exit(3);
      }
    }
    if (waitForEnabled || stopOnDisabled) {
      for (int i = 0; i < topicNames.length; i++) {
        if (topicNames[i].equals(enabledTopic)) {
          enabledIndex = i;
          break;
        }
      }
      if (enabledIndex < 0) {
        System.err.println("--enabled-topic must also be provided as a --topic: " + enabledTopic);
        inst.close();
        System.exit(4);
      }
    }
    List<SampleRow> spikes = new ArrayList<>();
    boolean hasSeenEnabled = false;
    long sampleStart = 0L;
    while (System.nanoTime() < end) {
      String[] values = new String[subscribers.length];
      NetworkTableValue[] sampledValues = new NetworkTableValue[subscribers.length];
      for (int i = 0; i < subscribers.length; i++) {
        NetworkTableValue value = subscribers[i].get();
        sampledValues[i] = value;
        summaries[i].add(value);
        values[i] = valueToText(value);
      }
      boolean enabled = false;
      if (enabledIndex >= 0) {
        NetworkTableValue enabledValue = subscribers[enabledIndex].get();
        enabled = enabledValue != null && enabledValue.isValid() && enabledValue.isBoolean() && enabledValue.getBoolean();
      }
      if (waitForEnabled && !hasSeenEnabled) {
        if (enabled) {
          hasSeenEnabled = true;
          sampleStart = System.nanoTime();
        } else {
          Thread.sleep(sleepMillis);
          continue;
        }
      } else if (sampleStart == 0L) {
        sampleStart = System.nanoTime();
      }
      if (spikeIndex >= 0) {
        NetworkTableValue spikeValue = sampledValues[spikeIndex];
        boolean hasSpikeSample = false;
        double spikeSample = 0.0;
        if (spikeValue != null && spikeValue.isValid()) {
          if (spikeValue.isDouble()) {
            spikeSample = spikeValue.getDouble();
            hasSpikeSample = true;
          } else if (spikeValue.isInteger()) {
            spikeSample = spikeValue.getInteger();
            hasSpikeSample = true;
          }
        }
        if (hasSpikeSample && spikeSample >= spikeThreshold) {
          spikes.add(new SampleRow(System.currentTimeMillis() / 1000.0, values));
        }
      }
      if (stopOnDisabled && hasSeenEnabled && !enabled) {
        break;
      }
      Thread.sleep(sleepMillis);
    }
    double sampledSeconds = sampleStart == 0L ? 0.0 : (System.nanoTime() - sampleStart) / 1_000_000_000.0;

    if (json) {
      printJson(host, port, sampledSeconds, period, topicNames, summaries, spikes);
    } else {
      System.out.println("Connected to " + host + ":" + port);
      System.out.printf("Sampled %.3fs at %.3fs period%n", sampledSeconds, period);
      for (int i = 0; i < topicNames.length; i++) {
        Summary s = summaries[i];
        String line = topicNames[i] + "\tsamples=" + s.validSamples + "\tfirst=" + s.first + "\tlatest=" + s.latest;
        if (s.numeric && s.validSamples > 0) {
          line += "\tmin=" + s.min + "\tmax=" + s.max;
        }
        System.out.println(line);
      }
      if (spikeIndex >= 0) {
        System.out.println("Spikes where " + spikeTopic + " >= " + spikeThreshold + ": " + spikes.size());
        for (SampleRow spike : spikes) {
          StringBuilder line = new StringBuilder(String.format("spike timestamp=%.3f", spike.timestamp));
          for (int i = 0; i < topicNames.length; i++) {
            line.append("\t").append(topicNames[i]).append("=").append(spike.values[i]);
          }
          System.out.println(line);
        }
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
    """Sample live NetworkTables data and detect spike anomalies."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="10.80.44.2", help="Robot IP or hostname")
    parser.add_argument("--port", type=int, default=5810, help="NT4 port")
    parser.add_argument("--timeout", type=float, default=5.0, help="Connection timeout in seconds")
    parser.add_argument("--duration", type=float, default=5.0, help="Sampling duration in seconds")
    parser.add_argument("--period", type=float, default=0.1, help="Sampling period in seconds")
    parser.add_argument("--topic", action="append", required=True, help="Topic to sample; repeatable")
    parser.add_argument("--spike-topic", default="", help="Topic to compare against --spike-threshold; must also be passed as --topic")
    parser.add_argument("--spike-threshold", type=float, default=0.0, help="Print raw samples where --spike-topic is at least this value")
    parser.add_argument("--wait-enabled", action="store_true", help="Wait to start the summary window until --enabled-topic is true")
    parser.add_argument("--until-disabled", action="store_true", help="Stop sampling after --enabled-topic goes false after being true")
    parser.add_argument("--enabled-topic", default="/AdvantageKit/DriverStation/Enabled", help="Boolean DriverStation enabled topic")
    parser.add_argument("--json", action="store_true", help="Print JSON summary")
    args = parser.parse_args()

    raise SystemExit(
        compile_and_run(
            JAVA_SOURCE,
            "SampleLiveNt",
            [
                args.host,
                str(args.port),
                str(args.timeout),
                str(args.duration),
                str(args.period),
                str(args.json).lower(),
                str(args.spike_threshold),
                args.spike_topic,
                str(args.wait_enabled).lower(),
                str(args.until_disabled).lower(),
                args.enabled_topic,
                *args.topic,
            ],
        )
    )


if __name__ == "__main__":
    main()
