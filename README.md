# ASR-DeliverPackage
Repo for the ASR final project. Our project consists of making the Kobuki deliver packages to people.

The nodes and files modified by us to carry out the practice:
- In hri:
   - _hri_dependencies.launch.py_: to improve the response times we replaced the llama and whisper models by some lighter ones.  
   - _confirm.xml_ (The behavior tree of our project)
   - _DialogConfirmation.hpp_ and _DialogConfirmation.cpp_
   - _Listen.hpp and Listen.cpp_

## Behavior Tree
_**General idea:**_
![schema](https://github.com/user-attachments/assets/23051305-c3a6-4b6e-b39e-a3fcb0fee8ca)

_**Actual schema:**_
```xml
<?xml version="1.0"?>
<root main_tree_to_execute="BehaviorTree">
    <!-- ////////// -->
    <BehaviorTree ID="BehaviorTree">
        <Sequence>
            <RetryUntilSuccessful num_attempts="-1">
                <Action ID="Listen"/>
            </RetryUntilSuccessful>
            <Action ID="Speak" params="" speech_text="Buenas! Soy Kobuki, tu robot de reparto!"/>
            <Action ID="Speak" params="" speech_text="Dime dónde quieres que te lleve el paquete, los puntos disponibles son: Juanca que es la mesa de juanca, Profe que es la mesa del profesor, o Aseo que son los aseos del laboratorio piso uno"/>
            <RetryUntilSuccessful num_attempts="-1">
                <Fallback>
                    <Action ID="DialogConfirmation"  heard_text="{destino}" language="es" mode="set_dest" cordx="{x}" cordy="{y}" password="" pswrd=""/>
                    <Inverter>
                        <Action ID="Speak" params="" speech_text="No es un punto válido"/>
                    </Inverter>
                </Fallback>
            </RetryUntilSuccessful>
            <RetryUntilSuccessful num_attempts="-1">
                <Sequence>
                    <Action ID="Speak" params="" speech_text="OK, ahora necesito tu nombre y apellido, se usaran como remitente del envío"/>
                    <Fallback>
                        <Action ID="DialogConfirmation" cordx="" cordy="" language="es" mode="set_remitent" password="" heard_text="{remitent}"/>
                        <Inverter>
                            <Action ID="Speak" params="" speech_text="No es un nombre y apellido válido, deben ser dos palabras"/>
                        </Inverter>
                    </Fallback>
                    <Action ID="Speak" params="{remitent}" speech_text="Confirmame si es correcto, he escuchado : []"/>
                    <Fallback>
                        <Action ID="DialogConfirmation" cordx="" cordy="" heard_text="" language="es" mode="receive/give_pkg" password=""/>
                        <Inverter>
                            <Action ID="Speak" params="" speech_text="volvamos a intentarlo"/>
                        </Inverter>
                    </Fallback>
                </Sequence>
            </RetryUntilSuccessful>
            <RetryUntilSuccessful num_attempts="-1">
                <Sequence>
                    <Action ID="Speak" params="" speech_text="Por favor dime la contraseña que quieres para tu envio"/>
                    <Fallback>
                        <Action ID="DialogConfirmation" cordx="" cordy="" language="es" mode="set_password" password="" pswrd="{password}"/>
                        <Inverter>
                            <Action ID="Speak" params="" speech_text="La contraseña debe ser de una palabra"/>
                        </Inverter>
                    </Fallback>
                    <Action ID="Speak" params="{password}" speech_text="Confirmame si esta es la contraseña que quieres usar, he escuchado : []"/>
                    <Fallback>
                        <Action ID="DialogConfirmation" cordx="" cordy="" heard_text="" language="es" mode="receive/give_pkg" password=""/>
                        <Inverter>
                            <Action ID="Speak" params="" speech_text="volvamos a intentarlo"/>
                        </Inverter>
                    </Fallback>
                </Sequence>
            </RetryUntilSuccessful>
            <Action ID="Speak" params="" speech_text=" ok entonce resumen del envío"/>
            <Action ID="Speak" params="{destino}" speech_text="Tu paquete irá a: []"/>
            <Action ID="Speak" params="{remitent}" speech_text="El remitente  sera: []"/>
            <Action ID="Speak" params="{password}" speech_text="La clave del envío será: []"/>
            <Action ID="Speak" params="" speech_text="Ya casi estamos, necesito que deposites con cuidado el paquete encima mía"/>
            <RetryUntilSuccessful num_attempts="-1">
                <Sequence>
                    <Action ID="Speak" params="" speech_text="¿Lo has hecho ya?"/>
                    <Fallback>
                        <Action ID="DialogConfirmation" cordx="" cordy="" heard_text="" language="es" mode="receive/give_pkg" password="" pswrd=""/>
                        <Inverter>
                            <Action ID="Speak" params="" speech_text="Por favor, necesito el paquete para entregarlo"/>
                        </Inverter>
                    </Fallback>
                </Sequence>
            </RetryUntilSuccessful>
            <Action ID="Speak" params="" speech_text="Perfecto! Me aseguraré de entregarlo! Adiós"/>
            <RetryUntilSuccessful num_attempts="3">
                <Fallback>
                    <Action ID="NavigateTo" distance_tolerance="0.1" tf_frame="" will_finish="true" x="{x}" y="{y}"/>
                    <Inverter>
                        <Action ID="Speak" params="" speech_text="Vaya,parece que hubo un error,volveré a la central para intentarlo de nuevo"/>
                    </Inverter>
                    <Inverter>
                        <Action ID="NavigateTo" distance_tolerance="0.05" tf_frame="" will_finish="true" x="{0.0}" y="{0.0}"/>
                    </Inverter>
                </Fallback>
            </RetryUntilSuccessful>
            <Fallback>
                <Sequence>
                    <Fallback>
                        <RetryUntilSuccessful num_attempts="5">
                            <Sequence>
                                <Action ID="Speak" params="{remitent}" speech_text="Tengo un paquete de parte de []"/>
                                <Action ID="Speak" params="" speech_text="si es para usted digame la contraseña del envio para recibirlo"/>
                                <Fallback>
                                    <Action ID="DialogConfirmation" cordx="" cordy="" heard_text="" language="es" mode="check_password" password="{password}" pswrd=""/>
                                    <Inverter>
                                        <Action ID="Speak" params="" speech_text="No coincide la contraseña"/>
                                    </Inverter>
                                </Fallback>
                            </Sequence>
                        </RetryUntilSuccessful>
                        <Inverter>
                            <Action ID="Speak" params="" speech_text="Numero maximo de intentos alcanzado,volviendo a la central"/>
                        </Inverter>
                    <!--Con will_finish fale intentará de manera indefinida-->
                    <Action ID="NavigateTo" distance_tolerance="0.0" tf_frame="" will_finish="false" x="0.0" y="0.0"/>
                    </Fallback>
                    <Action ID="Speak" params="" speech_text="Perfecto!Retira el paquete de encima mia"/>
                    <RetryUntilSuccessful num_attempts="-1">
                        <Sequence>
                            <Action ID="Speak" params="" speech_text="¿Lo has hecho ya?"/>
                            <Fallback>
                                <Action ID="DialogConfirmation" cordx="" cordy="" heard_text="" language="es" mode="receive/give_pkg" password="" pswrd=""/>
                                <Inverter>
                                    <Action ID="Speak" params="" speech_text="Por favor, necesito que retires el paquete"/>
                                </Inverter>
                            </Fallback>
                        </Sequence>
                    </RetryUntilSuccessful>
                    <Action ID="Speak" params="" speech_text="Genial,gracias por confiar en mi para recibir tu envio."/>
                    <Action ID="NavigateTo" distance_tolerance="0.0" tf_frame="" will_finish="false" x="0.0" y="0.0"/>
                </Sequence>
                <Action ID="NavigateTo" distance_tolerance="0.0" tf_frame="" will_finish="false" x="0.0" y="0.0"/>
            </Fallback>
        </Sequence>
    </BehaviorTree>
    <!-- ////////// -->
    <TreeNodesModel>
        <Action ID="DialogConfirmation">
            <output_port name="cordx"/>
            <output_port name="cordy"/>
            <output_port name="heard_text"/>
            <input_port default="en" name="language"/>
            <input_port name="mode"/>
            <input_port name="password"/>
            <output_port name="pswrd"/>
        </Action>
        <Action ID="Listen">
            <output_port name="listened_text">What was heard</output_port>
        </Action>
        <Action ID="NavigateTo">
            <input_port name="distance_tolerance"/>
            <input_port name="tf_frame"/>
            <input_port name="will_finish"/>
            <input_port name="x"/>
            <input_port name="y"/>
        </Action>
        <Action ID="Speak">
            <input_port name="params"/>
            <input_port name="speech_text"/>
        </Action>
    </TreeNodesModel>
    <!-- ////////// -->
</root>
```

### BT's Description
This is the Kobuki's delivery workflow:
1) **Setup Delivery**: The Kobuki receives all the necessary information (_destination_ and _receiver_) and initializes a delivery operation.
   - We check if the destination name is registered in the list of waypoints.
   - The expected information about the receiver is their first name and last name.

2) **Execute Delivery**: Carries out the actual delivery process through two main phases:
   - **Travel Phase:** The robot attempts to travel to the specified destination. If that fails, it returns to the home base.
     - The waypoint should be at the entrance of the destination, so when it arrives, it changes its state to DeliverConfirmation.
   - **Search and Deliver Phase:** After reaching the destination, the robot:
     - Tries up to 5 times (_num_attempts_) to:
       - Verify the actual receiver and deliver the package
       - Return to the home base
     - If all attempts fail (receiver not found), it returns to the home base
     - Once it reaches the destination, it starts loudly asking for the expected receiver’s name.
     - When someone responds, it asks for the password to verify if they are the correct person. If not, or if there’s a misunderstanding or no answer, it repeats the process several times. If the receiver is definitely not there, it returns to the base.
     - When it finds the receiver, it gives a green light to pick up the package and then returns to the base.
    
## Jaro-Warklers Algorithm
Since the Whisper model isn't perfect because it doesn't always hear exactly what was said, we've implemented an algorithm that compares the similarity between two words to avoid that problem.

```cpp
void DialogConfirmation::replaceAllOccurrences(std::string& str, const std::string& from, const std::string& to) {
    if (from.empty())
        return;
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length();
    }
}

std::string DialogConfirmation::normalizeString(const std::string& inputStr) {
    std::string currentStr = inputStr;

    replaceAllOccurrences(currentStr, "\xc3\x81", "a"); replaceAllOccurrences(currentStr, "\xc3\xa1", "a");
    replaceAllOccurrences(currentStr, "\xc3\x89", "e"); replaceAllOccurrences(currentStr, "\xc3\xa9", "e");
    replaceAllOccurrences(currentStr, "\xc3\x8d", "i"); replaceAllOccurrences(currentStr, "\xc3\xad", "i");
    replaceAllOccurrences(currentStr, "\xc3\x93", "o"); replaceAllOccurrences(currentStr, "\xc3\xb3", "o");
    replaceAllOccurrences(currentStr, "\xc3\x9a", "u"); replaceAllOccurrences(currentStr, "\xc3\xba", "u");
    replaceAllOccurrences(currentStr, "\xc3\x9c", "u"); replaceAllOccurrences(currentStr, "\xc3\xbc", "u");
    replaceAllOccurrences(currentStr, "\xc3\x91", "n"); replaceAllOccurrences(currentStr, "\xc3\xb1", "n");

    std::string fullyLoweredStr;
    fullyLoweredStr.reserve(currentStr.length());
    for (char ch_orig : currentStr) {
        fullyLoweredStr += static_cast<char>(std::tolower(static_cast<unsigned char>(ch_orig)));
    }

    std::string phoneticallyProcessedStr;
    phoneticallyProcessedStr.reserve(fullyLoweredStr.length());
    for (size_t i = 0; i < fullyLoweredStr.length(); ++i) {
        char currentChar = fullyLoweredStr[i];
        char charToAdd = 0;
        if (currentChar == 'h') {}
        else if (currentChar == 'v') { charToAdd = 'b'; }
        else if (currentChar == 'z') { charToAdd = 's'; }
        else if (currentChar == 'c') {
            char nextChar = (i + 1 < fullyLoweredStr.length()) ? fullyLoweredStr[i+1] : '\0';
            if (nextChar == 'e' || nextChar == 'i') { charToAdd = 's'; }
            else { charToAdd = 'k'; }
        } else if (currentChar == 'q') {
            if (i + 1 < fullyLoweredStr.length() && fullyLoweredStr[i+1] == 'u') {
                char charAfterU = (i + 2 < fullyLoweredStr.length()) ? fullyLoweredStr[i+2] : '\0';
                if (charAfterU == 'e' || charAfterU == 'i') { charToAdd = 'k'; i++; }
                else { charToAdd = 'k'; i++;}
            } else { charToAdd = 'k'; }
        } else { charToAdd = currentChar; }
        if (charToAdd != 0) phoneticallyProcessedStr += charToAdd;
    }

    std::string finalResultStr;
    finalResultStr.reserve(phoneticallyProcessedStr.length());
    for (char ch : phoneticallyProcessedStr) {
        if (std::isalnum(static_cast<unsigned char>(ch))) {
            finalResultStr += ch;
        }
    }
    return finalResultStr;
}

double DialogConfirmation::jaroSimilarity(const std::string& s1, const std::string& s2) {
    const int len1 = s1.length();
    const int len2 = s2.length();

    if (len1 == 0 || len2 == 0) {
        return (len1 == 0 && len2 == 0) ? 1.0 : 0.0;
    }

    int match_distance = std::max(0, static_cast<int>(std::floor(std::max(len1, len2) / 2.0)) - 1);

    std::vector<bool> s1_matches(len1, false);
    std::vector<bool> s2_matches(len2, false);

    int matches = 0;
    for (int i = 0; i < len1; ++i) {
        int start = std::max(0, i - match_distance);
        int end = std::min(i + match_distance + 1, len2);

        for (int j = start; j < end; ++j) {
            if (s2_matches[j]) continue;
            if (s1[i] != s2[j]) continue;

            s1_matches[i] = true;
            s2_matches[j] = true;
            matches++;
            break;
        }
    }

    if (matches == 0) {
        return 0.0;
    }

    std::string s1_matched_chars; s1_matched_chars.reserve(matches);
    for(int i=0; i < len1; ++i) if(s1_matches[i]) s1_matched_chars += s1[i];

    std::string s2_matched_chars; s2_matched_chars.reserve(matches);
    for(int i=0; i < len2; ++i) if(s2_matches[i]) s2_matched_chars += s2[i];

    int half_transpositions = 0;
    for (int i = 0; i < matches; ++i) {
        if (s1_matched_chars[i] != s2_matched_chars[i]) {
            half_transpositions++;
        }
    }
    int transpositions = half_transpositions / 2;

    double dj = ((double)matches / len1 +
                (double)matches / len2 +
                (double)(matches - transpositions) / matches) / 3.0;
    return dj;
}

double DialogConfirmation::jaroWinklerSimilarity(const std::string& s1, const std::string& s2, double p_scaling_factor, int max_prefix_length) {
    if (s1.empty() && s2.empty()) return 1.0;
    if (s1.empty() || s2.empty()) return 0.0;

    double jaro_dist = jaroSimilarity(s1, s2);

    if (jaro_dist == 0.0) return 0.0;

    int common_prefix_len = 0;
    for (int i = 0; i < std::min({(int)s1.length(), (int)s2.length(), max_prefix_length}); ++i) {
        if (s1[i] == s2[i]) {
            common_prefix_len++;
        } else {
            break;
        }
    }
    return jaro_dist + common_prefix_len * p_scaling_factor * (1.0 - jaro_dist);
}

bool DialogConfirmation::areSimilar(const std::string& str1, const std::string& str2, double threshold) {
    std::string normalizedStr1 = normalizeString(str1);
    std::string normalizedStr2 = normalizeString(str2);

    if (normalizedStr1.empty() && normalizedStr2.empty()) {
        return true;
    }

    double similarity = jaroWinklerSimilarity(normalizedStr1, normalizedStr2);

    return similarity >= threshold;
}
```

## Slides
[https://urjc-my.sharepoint.com/:p:/r/personal/jl_laria_2023_alumnos_urjc_es/Documents/Mechanic%20Gear%20Aesthetic%20Project%20Proposal%20by%20Slidesgo.pptx?d=w4c6765283742479fa64857ffe628d0ba&csf=1&web=1&e=Ntlo3U](url)
