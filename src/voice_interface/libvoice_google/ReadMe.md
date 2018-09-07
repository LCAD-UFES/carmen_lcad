# 1. Criando uma conta no Google Cloud Console

To begin to use a Google Cloud API, you must have a Google account.

  - Select or create a GCP project. [Go to the Manage Page Resources](https://console.cloud.google.com/cloud-resource-manager?_ga=2.159473469.-1617484999.1535991245)

  - Make sure that billing is enabled for your project. [Learn how to enable Billing](https://cloud.google.com/billing/docs/how-to/modify-project)

  - Enable the APIs. First, access the [search page](https://console.cloud.google.com/apis/library?project=voice-iara&folder&organizationId), select the project in the top bar, and search for "Cloud Text-to-Speech API". Click the banner and select activate in the next page. Repeat the process, but this time search for "Cloud Speech API" to activate the  Speech-to-Text API. 

  - Set up authentication:
  -- Go to the [Create Service Account Key](https://console.cloud.google.com/apis/credentials/serviceaccountkey?_ga=2.62067500.-1617484999.1535991245) page in the GCP Console. (*Menu(≡) ->  API&Services -> Credentials*)
-- From the Service account drop-down list, select *New service account*.
  -- Enter a name into the Service account name field. *(Tip: do not use spaces ' ')*
  -- Don't select a value from the Role drop-down list. No role is required to access this service. *(Tip: set Owner as role)*
  -- Click *Create*. (Perhaps, a note will appear, warning that this service account has no role).
  -- Click *Create without role*. A JSON file that contains your key downloads to your computer.

  - Set the environment variable GOOGLE_APPLICATION_CREDENTIALS to the file path of the JSON file that contains your service account key. This variable only applies to your current shell session, so if you open a new session, set the variable again. 
  > *e.g.
  ```sh
  export GOOGLE_APPLICATION_CREDENTIALS=/home/user/path/project_name-key.json)*
  ```
  
  - Install and initialize [Google Cloud SDK](https://cloud.google.com/sdk/docs/#deb)
  ```sh
  export CLOUD_SDK_REPO="cloud-sdk-$(lsb_release -c -s)"
  echo "deb http://packages.cloud.google.com/apt $CLOUD_SDK_REPO main" | sudo tee -a /etc/apt/sources.list.d/google-cloud-sdk.list
  curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
  sudo apt-get update && sudo apt-get install google-cloud-sdk
  ```
  Run the SDK to set configurations:
  ```sh
  gcloud init
  ```

# 2. Install the Client Library

Examples of the Text-to-Speech and Speech-to-Text APIs:

```sh
sudo apt-get install python-pyaudio vlc
pip install --upgrade google-cloud-texttospeech google-cloud-speech python-vlc
```

### 3. Some Python Samples

https://github.com/GoogleCloudPlatform/python-docs-samples

### 4. Tips

  - Add your directory as a PYTHONPATH:
  ```sh
  export PYTHONPATH=$PYTHONPATH:`pwd`
  ```
***

### See more about Google Cloud APIS

> Google Cloud APIs are a key part of Google Cloud Platform, allowing you to
> easily add the power of everything from storage access to machine-learning-based 
> image analysis to your Cloud Platform applications.
> https://cloud.google.com/apis/docs/overview
