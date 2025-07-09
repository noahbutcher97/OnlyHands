#if PLATFORM_IOS
#import <CoreHaptics/CoreHaptics.h>
#import <UIKit/UIKit.h>
#import <AudioToolbox/AudioToolbox.h>

// Make sure this struct matches the C++ definition exactly!
struct FOHHapticParameterCurve
{
    TArray<float> Times;
    TArray<float> Values;
};

struct FOHHapticEventEx
{
    float StartTime;
    float Duration;
    FOHHapticParameterCurve IntensityCurve;
    FOHHapticParameterCurve SharpnessCurve;
    bool bTransient;
};

// Helper to convert FOHHapticParameterCurve to CHHapticParameterCurve
static CHHapticParameterCurve* MakeParameterCurve(const FOHHapticParameterCurve& curve, NSString* paramID)
{
    if (curve.Times.Num() < 2 || curve.Values.Num() < 2 || curve.Times.Num() != curve.Values.Num())
        return nil;
    NSMutableArray<CHHapticParameterCurveControlPoint*>* points = [NSMutableArray array];
    for (int i = 0; i < curve.Times.Num(); ++i)
    {
        [points addObject:[[CHHapticParameterCurveControlPoint alloc]
            initWithRelativeTime:curve.Times[i]
                           value:curve.Values[i]]];
    }
    return [[CHHapticParameterCurve alloc]
        initWithParameterID:paramID
        controlPoints:points
        relativeTime:0];
}

// Main CoreHaptics player for pattern with curves
extern "C" void PlayiOSCoreHapticPatternEx(const FOHHapticEventEx* events, int numEvents)
{
    if (@available(iOS 13.0, *))
    {
        static CHHapticEngine* engine = nil;
        if (!engine)
        {
            NSError* error = nil;
            engine = [[CHHapticEngine alloc] initAndReturnError:&error];
            [engine startAndReturnError:nil];
        }
        NSMutableArray* chEvents = [NSMutableArray array];
        NSMutableArray* chCurves = [NSMutableArray array];

        for (int i = 0; i < numEvents; ++i)
        {
            const FOHHapticEventEx& evt = events[i];

            CHHapticEvent* e = nil;
            // Default: use curve value[0] if curve present, else 1.0
            float intensity = (evt.IntensityCurve.Values.Num() > 0) ? evt.IntensityCurve.Values[0] : 1.0;
            float sharpness = (evt.SharpnessCurve.Values.Num() > 0) ? evt.SharpnessCurve.Values[0] : 1.0;

            NSArray* params = @[
                [[CHHapticEventParameter alloc] initWithParameterID:CHHapticEventParameterIDHapticIntensity value:intensity],
                [[CHHapticEventParameter alloc] initWithParameterID:CHHapticEventParameterIDHapticSharpness value:sharpness]
            ];

            if (evt.bTransient)
            {
                e = [[CHHapticEvent alloc] initWithEventType:CHHapticEventTypeHapticTransient
                                                  parameters:params
                                                relativeTime:evt.StartTime];
            }
            else
            {
                e = [[CHHapticEvent alloc] initWithEventType:CHHapticEventTypeHapticContinuous
                                                  parameters:params
                                                relativeTime:evt.StartTime
                                                    duration:evt.Duration];
            }
            [chEvents addObject:e];

            // Add curves if present
            CHHapticParameterCurve* intCurve = MakeParameterCurve(evt.IntensityCurve, CHHapticEventParameterIDHapticIntensity);
            if (intCurve) [chCurves addObject:intCurve];
            CHHapticParameterCurve* sharpCurve = MakeParameterCurve(evt.SharpnessCurve, CHHapticEventParameterIDHapticSharpness);
            if (sharpCurve) [chCurves addObject:sharpCurve];
        }

        NSError* error = nil;
        CHHapticPattern* pattern = [[CHHapticPattern alloc]
            initWithEvents:chEvents parameters:chCurves error:&error];
        if (!pattern) return;
        id<CHHapticPatternPlayer> player = [engine createPlayerWithPattern:pattern error:&error];
        [player startAtTime:0 error:&error];
    }
}

extern "C" void PlayiOSCoreHapticSimple(float intensity, float sharpness, float duration, bool bTransient)
{
    if (@available(iOS 13.0, *))
    {
        static CHHapticEngine* engine = nil;
        if (!engine)
        {
            NSError* error = nil;
            engine = [[CHHapticEngine alloc] initAndReturnError:&error];
            [engine startAndReturnError:nil];
        }
        NSError* error = nil;
        CHHapticEvent* event = nil;
        NSArray* params = @[
            [[CHHapticEventParameter alloc] initWithParameterID:CHHapticEventParameterIDHapticIntensity value:intensity],
            [[CHHapticEventParameter alloc] initWithParameterID:CHHapticEventParameterIDHapticSharpness value:sharpness]
        ];
        if (bTransient)
        {
            event = [[CHHapticEvent alloc] initWithEventType:CHHapticEventTypeHapticTransient parameters:params relativeTime:0];
        }
        else
        {
            event = [[CHHapticEvent alloc] initWithEventType:CHHapticEventTypeHapticContinuous parameters:params relativeTime:0 duration:duration];
        }
        CHHapticPattern* pattern = [[CHHapticPattern alloc] initWithEvents:@[event] parameters:@[] error:&error];
        id<CHHapticPatternPlayer> player = [engine createPlayerWithPattern:pattern error:&error];
        [player startAtTime:0 error:&error];
    }
}

#endif
