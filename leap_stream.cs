using System;
using System.Diagnostics;
using System.Globalization;
using System.Runtime.InteropServices;

public static class LeapStream
{
    private const string Library = @"C:\Program Files\Ultraleap\LeapSDK\lib\x64\LeapC.dll";
    private const uint TrackingEvent = 0x100;
    private const uint Timeout = 0xE2010004;
    private const uint ConnectionLost = 2;

    // LeapC.h specifies one-byte packing. Only the prefixes read by this demo
    // are declared for the event and hand structs.
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    private struct Message
    {
        public uint Size, Type;
        public IntPtr EventData;
        public uint DeviceId;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    private struct TrackingFrame
    {
        public IntPtr Reserved;
        public long FrameId, Timestamp, TrackingFrameId;
        public uint HandCount;
        public IntPtr Hands;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    private struct Hand
    {
        public uint Id, Flags, Type;
        public float Confidence;
        public ulong VisibleTime;
        public float PinchDistance, GrabAngle, PinchStrength, GrabStrength;
        public float PalmX, PalmY, PalmZ;
    }

    [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
    private static extern uint LeapCreateConnection(IntPtr config, out IntPtr connection);
    [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
    private static extern uint LeapOpenConnection(IntPtr connection);
    [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
    private static extern uint LeapPollConnection(IntPtr connection, uint timeout, ref Message message);
    [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
    private static extern void LeapDestroyConnection(IntPtr connection);

    private static void Check(uint result, string operation)
    {
        if (result != 0)
            throw new InvalidOperationException(operation + " failed (0x" + result.ToString("X8") + ").");
    }

    public static void Run()
    {
        IntPtr connection;
        Check(LeapCreateConnection(IntPtr.Zero, out connection), "Creating a LeapC connection");
        try
        {
            Check(LeapOpenConnection(connection), "Opening the LeapC connection");
            var clock = Stopwatch.StartNew();
            var message = new Message { Size = (uint)Marshal.SizeOf(typeof(Message)) };
            long lastFrameAt = -500;
            long nextOutputAt = 0;
            Hand? latestHand = null;

            while (true)
            {
                uint result = LeapPollConnection(connection, 100, ref message);
                long now = clock.ElapsedMilliseconds;
                if (result != Timeout)
                {
                    Check(result, "Reading LeapC events");
                    if (message.Type == TrackingEvent)
                    {
                        var frame = Marshal.PtrToStructure<TrackingFrame>(message.EventData);
                        latestHand = frame.HandCount == 0
                            ? (Hand?)null
                            : Marshal.PtrToStructure<Hand>(frame.Hands);
                        lastFrameAt = now;
                    }
                    if (message.Type == ConnectionLost)
                        throw new InvalidOperationException("The Ultraleap tracking service disconnected.");
                }

                if (now < nextOutputAt)
                    continue;

                if (latestHand.HasValue && now - lastFrameAt < 500)
                {
                    Hand hand = latestHand.Value;
                    Console.WriteLine(string.Format(CultureInfo.InvariantCulture, "{0:R},{1:R},{2:R},{3:R}",
                        hand.PalmX, hand.PalmY, hand.PalmZ, hand.PinchStrength));
                }
                else
                {
                    Console.WriteLine("MISSING");
                }
                nextOutputAt = now + 200;
            }
        }
        finally
        {
            LeapDestroyConnection(connection);
        }
    }
}
