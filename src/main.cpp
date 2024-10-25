#include <sensesp/sensors/constant_sensor.h>
#include <sensesp/signalk/signalk_value_listener.h>
#include <sensesp/transforms/lambda_transform.h>
#include <sensesp/transforms/linear.h>

#include "sensesp/sensors/digital_input.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/debounce.h"
#include "sensesp/transforms/integrator.h"
#include "sensesp/transforms/truth_text.h"
#include "sensesp_app.h"
#include "sensesp_app_builder.h"
#undef DEBUG_DISABLED

using namespace sensesp;

/**
 * This example illustrates an anchor chain counter. Note that it
 * doesn't distinguish between chain being let out and chain being
 * taken in, so the intended usage is this: Press the button to make
 * sure the counter is at 0.0. Let out chain until the counter shows
 * the amount you want out. Once the anchor is set, press the button
 * again to reset the counter to 0.0. As you bring the chain in, the
 * counter will show how much you have brought in. Press the button
 * again to reset the counter to 0.0, to be ready for the next anchor
 * deployment.
 *
 * A bi-directional chain counter is possible, but this is not one.
 */

ReactESP app;

const uint8_t GYPSY_PIN = 22;
const uint8_t UP_PIN = 27;
const uint8_t DOWN_PIN = 35;

// String constants

String hostName = "WindlassSensor";
String networkSSID = "Bertie";
String netwerkPassword = "Ookie1234";

const int gypsyCounterReadDelay_ms = 1000;
const String gypsyCounterReadDelayConfigPath = "/chain_counter/read_delay";

const float gypsyCircumference_m = 0.32;
const String gypsyCircumferenceceConfigPath = "/gypsy/circumference";
const unsigned int debounceWindow_ms = 15;
const String debounceWindowMsConfigPath = "/gypsy/debounceWindow_ms";

void setup() {
    SetupSerialDebug(115200);

    SensESPAppBuilder builder;
    sensesp_app = builder.set_hostname(hostName)
                      ->set_wifi("Bertie", "Ookie1234")
                      //->set_wifi("StevenR", "ILoveIrene")
                      //->set_wifi("ApollosRest", "Apollo4Ever")
                      ->get_app();

    // Publish a link to the firmware on the Sensor UI status page 
    //
    UIOutput<String>* firmware_url_ui_output_ =
      new UIOutput<String>("Firmware URL", "https://github.com/smr547/WindlassSensor", "Software", 2100);


    pinMode(GYPSY_PIN, INPUT_PULLUP);
    pinMode(UP_PIN, INPUT_PULLDOWN);
    pinMode(DOWN_PIN, INPUT_PULLDOWN);

    auto* gypsySensor = new DigitalInputChange(GYPSY_PIN, INPUT_PULLUP, CHANGE);
    auto* debouncer =
        new Debounce(debounceWindow_ms, debounceWindowMsConfigPath);

    auto* chainAccumulator = new IntegratorT<int, int>(1, 0);
    auto* gypsyDirectionSensor =
        new LambdaTransform<boolean, int>([chainAccumulator](boolean input) {
            bool up_pin = digitalRead(UP_PIN);
            bool down_pin = digitalRead(DOWN_PIN);
            Serial.println("Pulse detected");

            if (input) {
                return 0;  // we count only the falling edge
            }

            Serial.print("Falling edge detected. UP: ");
            Serial.print(up_pin);
            Serial.print(" DOWN: ");
            Serial.print(down_pin);
            Serial.print(" Chain count=");
            Serial.println(chainAccumulator->get());

            if (up_pin & !down_pin) {
                if (chainAccumulator->get() > 0) {
                    return -1;  // gypsy driven up
                } else {
                    return 0;  // prevent accumulator going negative
                }
            }
            if (down_pin & !up_pin) return 1;     // gypsy driven down
            if (!(down_pin || up_pin)) return 1;  // freefall

            return 0;
        });

    /*
        auto directionSenseFunction = []() -> int {
            bool up_pin = digitalRead(UP_PIN);
            bool down_pin = digitalRead(DOWN_PIN);

            if (up_pin & !down_pin) return -1;    // gypsy driven up
            if (down_pin & !up_pin) return 1;     // gypsy driven down
            if (!(down_pin || up_pin)) return 1;  // freefall

            return 0;
        };
    */
    // auto gypsyDirectionSensor =
    //     new LambdaTransform<bool, int>(directionSenseFunction);

    auto* gypsyCountMultiplier =
        new Linear(gypsyCircumference_m, 0.0, gypsyCircumferenceceConfigPath);

    /**
     * chain_counter is connected to accumulator, which is connected to an
     * SKOutputNumber, which sends the final result to the indicated path on the
     * Signal K server. (Note that each data type has its own version of
     * SKOutput: SKOutputNumber for floats, SKOutputInt, SKOutputBool, and
     * SKOutputString.)
     */
    String sk_path = "navigation.anchor.rodeDeployed";
    String sk_path_config_path = "/rodeDeployed/sk";

    /**
     * There is no path for the amount of anchor rode deployed in the current
     * Signal K specification. By creating an instance of SKMetaData, we can
     * send a partial or full defintion of the metadata that other consumers of
     * Signal K data might find useful. (For example, Instrument Panel will
     * benefit from knowing the units to be displayed.) The metadata is sent
     * only the first time the data value is sent to the server.
     */
    SKMetadata* metadata = new SKMetadata();
    metadata->units_ = "m";
    metadata->description_ = "Anchor Rode Deployed";
    metadata->display_name_ = "Rode Deployed";
    metadata->short_name_ = "Rode Out";

    auto* skChainOutOutput =
        new SKOutputFloat(sk_path, sk_path_config_path, metadata);

    // string all of the producers and consumers together

    gypsySensor->connect_to(debouncer)
        ->connect_to(gypsyDirectionSensor)
        ->connect_to(chainAccumulator)
        ->connect_to(gypsyCountMultiplier)
        ->connect_to(skChainOutOutput);

    /**
     * DigitalInputChange monitors a physical button connected to BUTTON_PIN.
     * Because its interrupt type is CHANGE, it will emit a value when the
     * button is pressed, and again when it's released, but that's OK - our
     * LambdaConsumer function will act only on the press, and ignore the
     * release. DigitalInputChange looks for a change every read_delay ms, which
     * can be configured at read_delay_config_path in the Config UI.
     */
    int read_delay = 10;
    String up_read_delay_config_path = "/up_button_watcher/read_delay";
    auto* up_button_watcher = new DigitalInputChange(
        UP_PIN, INPUT_PULLDOWN, CHANGE, up_read_delay_config_path);

    /**
     * Create a DebounceInt to make sure we get a nice, clean signal from the
     * button. Set the debounce delay period to 15 ms, which can be configured
     * at debounce_config_path in the Config UI.
     */
    int up_debounce_delay = 15;
    String up_debounce_config_path = "/up_debounce/delay";
    auto* up_debounce =
        new DebounceInt(up_debounce_delay, up_debounce_config_path);

    /**
     * When the button is pressed (or released), it will call the lambda
     * expression (or "function") that's called by the LambdaConsumer. This is
     * the function - notice that it calls reset() only when the input is 1,
     * which indicates a button press. It ignores the button release. If your
     * button goes to GND when pressed, make it "if (input == 0)".
     */
    auto reset_function = [chainAccumulator](int input) {
        if (input == 1) {
            //   debugW("Reset button pressed");
            //   chainAccumulator->reset();  // Resets the output to 0.0
        }
    };

    /**
     * Create the LambdaConsumer that calls reset_function, Because
     DigitalInputChange
     * outputs an int, the version of LambdaConsumer we need is
     LambdaConsumer<int>.
     *
     * While this approach - defining the lambda function (above) separate from
     the
     * LambdaConsumer (below) - is simpler to understand, there is a more
     concise approach:
     *
      auto* button_consumer = new LambdaConsumer<int>([accumulator](int input) {
        if (input == 1) {
          accumulator->reset();
        }
      });

     *
    */
    auto* button_consumer = new LambdaConsumer<int>(reset_function);

    /* Connect the button_watcher to the debounce to the button_consumer. */
    up_button_watcher->connect_to(up_debounce)->connect_to(button_consumer);

    /* report the state of the UP button */

    auto* up_button_interpreter = new TruthToText();

    SKMetadata* up_button_metadata = new SKMetadata();
    up_button_metadata->units_ = "";
    up_button_metadata->description_ =
        "State of the remote anchor control up button";
    up_button_metadata->display_name_ = "Up button state";
    up_button_metadata->short_name_ = "Up Button";

    String up_button_state_path = "navigation.windless.remote.upbutton.state";
    String up_button_state_config_path = "/up_button_state/sk";
    SKMetadata up_button_state_metadata;

    auto* up_button_reporter = new SKOutputString(
        up_button_state_path, up_button_state_config_path, up_button_metadata);

    up_debounce->connect_to(up_button_interpreter)
        ->connect_to(up_button_reporter);

    /* report the state of the DOWN button */

    String down_read_delay_config_path = "/down_button_watcher/read_delay";
    auto* down_button_watcher = new DigitalInputChange(
        DOWN_PIN, INPUT_PULLDOWN, CHANGE, down_read_delay_config_path);

    /**
     * Create a DebounceInt to make sure we get a nice, clean signal from the
     * button. Set the debounce delay period to 15 ms, which can be configured
     * at debounce_config_path in the Config UI.
     */
    int down_debounce_delay = 15;
    String down_debounce_config_path = "/down_debounce/delay";
    auto* down_debounce =
        new DebounceInt(down_debounce_delay, down_debounce_config_path);

    auto* down_button_interpreter = new TruthToText();

    SKMetadata* down_button_metadata = new SKMetadata();
    down_button_metadata->units_ = "";
    down_button_metadata->description_ =
        "State of the remote anchor control down button";
    down_button_metadata->display_name_ = "Down button state";
    down_button_metadata->short_name_ = "down Button";

    String down_button_state_path =
        "navigation.windless.remote.downbutton.state";
    String down_button_state_config_path = "/down_button_state/sk";

    auto* down_button_reporter =
        new SKOutputString(down_button_state_path,
                           down_button_state_config_path, down_button_metadata);

    down_button_watcher->connect_to(down_debounce)
        ->connect_to(down_button_interpreter)
        ->connect_to(down_button_reporter);

    /*************************************************************************************
     * CHAIN LENGTH
     **************************************************************************************/

    // Length of Chain

    const float chainLength = 100.0;
    SKMetadata* chainLengthMetadata = new SKMetadata();
    metadata->units_ = "m";
    metadata->description_ = "Anchor chain length";
    metadata->display_name_ = "Chain length";
    metadata->short_name_ = "Chain length";

    auto* chainLengthConstant =
        new ConstantSensor<float>(chainLength, 30, "/chain/length");
    auto* chainLengthReporter =
        new SKOutputFloat("navigation.anchor.chainLength", chainLengthMetadata);

    chainLengthConstant->connect_to(chainLengthReporter);

    /*************************************************************************************
     * Scope
     **************************************************************************************/

    auto* depthListener = new SKValueListener<float>("navigation.depth", 1000);

    auto* scopeCalculator =
        new LambdaTransform<float, float>([depthListener](float input) {
            float depth = depthListener->get();
            if (depth < 0.001) return 0.0;
            return (double)(input / depth);
        });

    skChainOutOutput->connect_to(scopeCalculator)
        ->connect_to(new SKOutputFloat("navigation.anchor.scope"));

    /*************************************************************************************
     * TEST CODE
     **************************************************************************************/

    // Depth of water shown here to allow scope to be reported

    const float depth = 5.1;

    auto* depthConstant =
        new ConstantSensor<float>(depth, 1, "/navigation/depth");
    auto* depthReporter = new SKOutputFloat("navigation.depth");

    depthConstant->connect_to(depthReporter);

    // Magnetic Heading (Degrees)

    const float headingDegrees = 45.0;

    auto* headingConstant = new ConstantSensor<float>(
        headingDegrees, 1, "/navigation/headingDegrees");
    auto* degToRadians = new Linear(3.142 / 180.0, 0);
    auto* headingReporter = new SKOutputFloat("navigation.headingMagnetic");

    headingConstant->connect_to(degToRadians)->connect_to(headingReporter);

    /* Finally, start the SensESPApp */
    sensesp_app->start();
}

// The loop function is called in an endless loop during program execution.
// It simply calls `app.tick()` which will then execute all reactions as needed.
void loop() { app.tick(); }
